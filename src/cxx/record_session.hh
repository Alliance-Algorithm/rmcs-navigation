#pragma once

#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace rmcs::navigation::details {

// All process and filesystem operations belong to the worker. The update thread
// only exchanges requests and cached snapshots, using try_lock rather than waiting.
class RecordSession {
public:
    struct Snapshot {
        bool ready = false;
        bool active = false;
        std::string status = "idle";
    };

    RecordSession(
        std::filesystem::path script, std::filesystem::path environment,
        std::filesystem::path state_directory);
    ~RecordSession();

    auto snapshot() -> const Snapshot&;
    auto start() -> std::pair<bool, std::string>;
    auto finish() -> std::pair<bool, std::string>;
    auto abort() -> std::pair<bool, std::string>;

private:
    enum class Stop { None, Finish, Abort };
    auto request_stop(Stop stop) -> std::pair<bool, std::string>;
    auto command(const char* operation) const -> bool;
    auto run() -> void;

    std::filesystem::path script_;
    std::filesystem::path environment_;
    std::filesystem::path state_directory_;
    std::mutex mutex_;
    std::condition_variable wake_;
    Snapshot shared_;
    Snapshot cached_; // Accessed only by the update thread.
    bool enabled_ = false;
    bool exiting_ = false;
    bool start_pending_ = false;
    Stop stop_pending_ = Stop::None;
    std::thread worker_;
};

} // namespace rmcs::navigation::details
