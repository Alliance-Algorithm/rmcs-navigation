#include "cxx/record_session.hh"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <unistd.h>

using rmcs::navigation::details::RecordSession;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

auto require(bool value, const char* message) -> void {
    if (!value)
        throw std::runtime_error{message};
}

auto main() -> int {
    auto root = std::filesystem::temp_directory_path()
                / ("rmcs record worker test " + std::to_string(getpid()));
    std::filesystem::create_directories(root);
    auto script = root / "slow control.sh";
    std::ofstream{script} << R"SH(#!/usr/bin/env bash
set -eu
op=$1
state=$3
touch "$state/$op-entered"
sleep 0.4
case "$op" in
  probe) test -f "$state/active" ;;
  start)
    test ! -f "$state/fail-start"
    touch "$state/active"
    printf 'recording test-bag\n' >"$state/record.status"
    ;;
  stop) printf 'mapping test-bag\n' >"$state/record.status" ;;
  abort)
    rm -f "$state/active"
    printf 'aborted\n' >"$state/record.status"
    ;;
esac
)SH";
    auto maximum = Clock::duration::zero();
    auto poll = [&](RecordSession& session) {
        auto before = Clock::now();
        auto snapshot = session.snapshot();
        maximum = std::max(maximum, Clock::now() - before);
        require(maximum < 50ms, "snapshot waited for an external command");
        return snapshot;
    };
    auto await = [&](auto predicate, const char* message) {
        auto deadline = Clock::now() + 6s;
        while (!predicate()) {
            require(Clock::now() < deadline, message);
            std::this_thread::sleep_for(1ms);
        }
    };
    auto request = [&](auto function) {
        auto before = Clock::now();
        auto result = function();
        maximum = std::max(maximum, Clock::now() - before);
        require(maximum < 50ms, "request waited for an external command");
        return result;
    };
    {
        auto session = RecordSession{script, root / "env with spaces.bash", root};
        std::this_thread::sleep_for(50ms);
        require(!std::filesystem::exists(root / "probe-entered"), "unused manager ran a command");
        require(!poll(session).ready, "initial probe should be asynchronous");
        await([&] { return poll(session).ready; }, "initial probe did not complete");
        require(!poll(session).active, "idle manager claims workflow exists");
        require(request([&] { return session.start(); }).first, "start was rejected");
        require(poll(session).status == "starting", "stale terminal state leaked after start");
        require(poll(session).active, "pending launch was not reserved");
        require(!request([&] { return session.start(); }).first, "duplicate start accepted");
        // A stop accepted before the slow start finishes must survive dispatch.
        require(request([&] { return session.finish(); }).first, "early finish was rejected");
        await([&] { return poll(session).status == "mapping test-bag"; }, "early finish was lost");
        require(!request([&] { return session.finish(); }).first, "mapping was stopped twice");
        require(request([&] { return session.abort(); }).first, "abort was rejected");
        await([&] { return poll(session).status == "aborted"; }, "abort was lost");
        await([&] { return !poll(session).active; }, "dead workflow still active");

        std::ofstream{root / "fail-start"} << "fail";
        require(request([&] { return session.start(); }).first, "failure launch was rejected");
        await([&] { return poll(session).status.starts_with("failed:"); }, "launch failure not reported");
        auto deadline = Clock::now() + 600ms;
        while (Clock::now() < deadline) {
            require(poll(session).status.starts_with("failed:"), "launch failure overwritten by stale disk state");
            std::this_thread::sleep_for(1ms);
        }
        std::filesystem::remove(root / "fail-start");
        require(request([&] { return session.start(); }).first, "retry launch was rejected");
        await([&] { return poll(session).status == "recording test-bag"; }, "retry did not start");
    }
    // The independent workflow remains active when its frontend is destroyed.
    {
        auto session = RecordSession{script, root / "env with spaces.bash", root};
        await([&] { return poll(session).ready; }, "restart probe did not complete");
        require(poll(session).active, "surviving workflow was not adopted");
        require(poll(session).status == "recording test-bag", "adopted workflow lost its stage");
        require(!request([&] { return session.start(); }).first, "adoption started a duplicate");
        require(request([&] { return session.abort(); }).first, "adopted workflow could not be aborted");
        await([&] { return poll(session).status == "aborted"; }, "adopted abort lost");
    }
    std::filesystem::remove_all(root);
    std::cout << "record_session: slow start/probe/stop/abort never blocked the caller; max "
              << std::chrono::duration<double, std::milli>{maximum}.count() << " ms\n";
}
