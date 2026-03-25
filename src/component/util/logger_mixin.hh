#pragma once

#include <format>
#include <utility>

#include <rclcpp/logging.hpp>

namespace rmcs {

struct LoggerMixin {
    template <typename Self, typename... Args>
    auto info(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_INFO(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto warn(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_WARN(self.get_logger(), "%s", text.c_str());
    }

    template <typename Self, typename... Args>
    auto error(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.get_logger(), "%s", text.c_str());
    }
};

} // namespace rmcs
