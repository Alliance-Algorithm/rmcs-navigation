#pragma once

#include <cstdlib>
#include <format>
#include <functional>
#include <string>
#include <thread>

namespace rmcs::navigation {

class NavigationScreen {
public:
    using Logger = std::function<void(const std::string&)>;

    explicit NavigationScreen(Logger logger)
        : logger_{std::move(logger)} {}

    auto set_config_name(std::string config_name) -> void { config_name_ = std::move(config_name); }

    auto start() const -> void {
        const auto source_command = std::string{"source /root/env_setup.bash"};
        const auto launch_command = std::format(
            "ros2 launch rmcs-navigation online.launch.py config_name:={}", config_name_);

        const auto screen_label = std::string{"rmcs-navigation"};
        const auto start_cmd = std::format(
            "screen -dmS {} bash -lc \"{} && {}\"", screen_label, source_command, launch_command);

        if (std::system(start_cmd.c_str()) != 0) {
            log("  [!] Error: Failed to execute screen start command.");
        }
    }

    static auto close() -> void {
        const auto screen_label = std::string{"rmcs-navigation"};
        const auto stop_cmd = std::format("screen -S {} -X quit", screen_label);
        std::ignore = std::system(stop_cmd.c_str());
    }

    static auto running() -> bool {
        const auto screen_label = std::string{"rmcs-navigation"};
        const auto check_cmd =
            std::format("screen -S {} -Q select . >/dev/null 2>&1", screen_label);
        return std::system(check_cmd.c_str()) == 0;
    }

    auto restart_async() const -> void {
        std::thread([this] { restart_navigation(); }).detach();
    }

private:
    auto log(const std::string& message) const -> void {
        if (logger_) {
            logger_(message);
        }
    }

    auto restart_navigation() const -> void {
        using namespace std::chrono_literals;

        const auto screen_label = std::string{"rmcs-navigation"};
        log(std::format("[Navigation] Target: {}", screen_label));

        if (running()) {
            log("  -> Found existing session, terminating...");
            close();
            std::this_thread::sleep_for(500ms);
        }

        log("  -> Starting new navigation instance...");
        start();

        std::this_thread::sleep_for(800ms);
        if (running()) {
            log("  [OK] Navigation is now running in background.");
        } else {
            log("  [X] Critical: Screen died immediately after start.");
            log(std::format("      Config name tried: {}", config_name_));
        }
    }

    Logger logger_;
    std::string config_name_ = "rmul";
};

} // namespace rmcs::navigation
