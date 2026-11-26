#include "cxx/record_session.hh"

#include <array>
#include <cerrno>
#include <chrono>
#include <fstream>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace rmcs::navigation::details {

RecordSession::RecordSession(
    std::filesystem::path script, std::filesystem::path environment,
    std::filesystem::path state_directory)
    : script_{std::move(script)}
    , environment_{std::move(environment)}
    , state_directory_{std::move(state_directory)}
    , worker_{[this] { run(); }} {}

RecordSession::~RecordSession() {
    {
        auto lock = std::lock_guard{mutex_};
        exiting_ = true;
    }
    wake_.notify_one();
    worker_.join();
}

auto RecordSession::snapshot() -> const Snapshot& {
    auto lock = std::unique_lock{mutex_, std::try_to_lock};
    if (lock.owns_lock()) {
        if (!enabled_) {
            enabled_ = true;
            wake_.notify_one();
        }
        cached_ = shared_;
    }
    return cached_;
}

auto RecordSession::start() -> std::pair<bool, std::string> {
    snapshot();
    auto lock = std::unique_lock{mutex_, std::try_to_lock};
    if (!lock.owns_lock())
        return {false, "record manager busy; retry"};
    if (!shared_.ready)
        return {false, "record manager initializing"};
    if (shared_.active)
        return {false, "record/map workflow already running"};
    shared_.active = true;
    shared_.status = "starting";
    start_pending_ = true;
    stop_pending_ = Stop::None;
    cached_ = shared_;
    wake_.notify_one();
    return {true, "ok"};
}

auto RecordSession::request_stop(Stop stop) -> std::pair<bool, std::string> {
    auto lock = std::unique_lock{mutex_, std::try_to_lock};
    if (!lock.owns_lock())
        return {false, "record manager busy; retry"};
    if (!shared_.active)
        return {false, "no record/map workflow"};
    if (stop == Stop::Finish && shared_.status != "starting"
        && !shared_.status.starts_with("recording "))
        return {false, "record/map workflow is not recording"};
    if (stop_pending_ != Stop::Abort)
        stop_pending_ = stop;
    wake_.notify_one();
    return {true, "ok"};
}

auto RecordSession::finish() -> std::pair<bool, std::string> {
    return request_stop(Stop::Finish);
}

auto RecordSession::abort() -> std::pair<bool, std::string> {
    return request_stop(Stop::Abort);
}

auto RecordSession::command(const char* operation) const -> bool {
    // posix_spawn avoids both a fork in the control thread and system()'s
    // process-wide changes to SIGINT/SIGQUIT while waiting for a shell.
    auto arguments = std::array{
        std::string{"bash"}, script_.string(), std::string{operation},
        environment_.string(), state_directory_.string()};
    auto argv = std::array<char*, 6>{};
    for (auto i = std::size_t{0}; i < arguments.size(); ++i)
        argv[i] = arguments[i].data();

    auto actions = posix_spawn_file_actions_t{};
    if (posix_spawn_file_actions_init(&actions) != 0)
        return false;
    auto log = (state_directory_ / "record-control.log").string();
    auto result = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    if (result == 0)
        result = posix_spawn_file_actions_addopen(
            &actions, STDOUT_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0600);
    if (result == 0)
        result = posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    auto pid = pid_t{};
    if (result == 0)
        result = posix_spawnp(&pid, "bash", &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (result != 0)
        return false;
    auto status = int{};
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR)
            return false;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

auto RecordSession::run() -> void {
    using Clock = std::chrono::steady_clock;
    auto next_probe = Clock::time_point{};
    auto active = false;
    auto failure = std::string{};
    while (true) {
        auto start = false;
        auto stop = Stop::None;
        {
            auto lock = std::unique_lock{mutex_};
            wake_.wait_for(lock, std::chrono::milliseconds{200}, [this] {
                return exiting_ || start_pending_ || stop_pending_ != Stop::None
                       || (enabled_ && !shared_.ready);
            });
            if (exiting_)
                return;
            if (!enabled_)
                continue;
            start = std::exchange(start_pending_, false);
            stop = std::exchange(stop_pending_, Stop::None);
        }

        auto status = std::string{};
        try {
            std::filesystem::create_directories(state_directory_);
            if (start)
                failure.clear();
            if (start && !command("start"))
                status = "failed: cannot start record workflow; see record-control.log";
            if (stop != Stop::None && !command(stop == Stop::Abort ? "abort" : "stop"))
                status = "failed: cannot request record shutdown; see record-control.log";
            if (start || Clock::now() >= next_probe) {
                active = command("probe");
                next_probe = Clock::now() + std::chrono::seconds{2};
            }
            if (!status.empty())
                failure = status;
            if (!failure.empty()) {
                status = failure;
            } else {
                auto file = std::ifstream{state_directory_ / "record.status"};
                if (!std::getline(file, status) || status.empty())
                    status = active ? "starting" : "idle";
            }
        } catch (const std::exception& error) {
            status = std::string{"failed: record manager: "} + error.what();
            failure = status;
            active = false;
        }
        auto lock = std::lock_guard{mutex_};
        // A request accepted during an idle probe must not be overwritten by
        // the older disk state before its launch is processed.
        if (!start_pending_)
            shared_ = {.ready = true, .active = active, .status = std::move(status)};
    }
}

} // namespace rmcs::navigation::details
