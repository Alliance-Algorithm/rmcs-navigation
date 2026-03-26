#pragma once

#include <cstdlib>
#include <format>
#include <functional>
#include <string>
#include <thread>

namespace rmcs::navigation {

class NavigationRestarter {
public:
    using Logger = std::function<void(const std::string&)>;

    explicit NavigationRestarter(Logger logger)
        : logger_{std::move(logger)} {}

    auto start_async(std::string config_name) const -> void {
        std::thread([this, config_name = std::move(config_name)] {
            restart_navigation(config_name);
        }).detach();
    }

private:
    auto log(const std::string& message) const -> void {
        if (logger_) {
            logger_(message);
        }
    }

    auto restart_navigation(const std::string& config_name) const -> void {
        using namespace std::chrono_literals;

        const auto source_command = std::string{"source /root/env_setup.bash"};
        const auto launch_command = std::format(
            "ros2 launch rmcs-navigation online.launch.py config_name:={}", config_name);

        const auto screen_label = std::string{"rmcs-navigation"};

        const auto check_cmd =
            std::format("screen -S {} -Q select . >/dev/null 2>&1", screen_label);
        const auto stop_cmd = std::format("screen -S {} -X quit", screen_label);
        const auto start_cmd = std::format(
            "screen -dmS {} bash -lc \"{} && {}\"", screen_label, source_command, launch_command);

        log(std::format("[Navigation] Target: {}", screen_label));

        if (std::system(check_cmd.c_str()) == 0) {
            log("  -> Found existing session, terminating...");
            std::ignore = std::system(stop_cmd.c_str());
            std::this_thread::sleep_for(500ms);
        }

        log("  -> Starting new navigation instance...");
        if (std::system(start_cmd.c_str()) != 0) {
            log("  [!] Error: Failed to execute screen start command.");
            return;
        }

        std::this_thread::sleep_for(800ms);
        if (std::system(check_cmd.c_str()) == 0) {
            log("  [OK] Navigation is now running in background.");
        } else {
            log("  [X] Critical: Screen died immediately after start.");
            log(std::format("      Command tried: {}", launch_command));
        }
    }

    Logger logger_;
};

} // namespace rmcs::navigation
