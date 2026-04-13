#pragma once

#include <format>
#include <utility>

#include <rclcpp/logging.hpp>

namespace rmcs::navigation {

struct NodeMixin {
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
    auto fuck(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.get_logger(), "%s", text.c_str());
    }

    template <typename T>
    auto param(this const auto& self, const std::string& name) -> T {
        if (self.has_parameter(name))
            return self.template get_parameter_or<T>(name, T{});

        self.fuck("param [ {} ] for {} is needed", name, self.get_name());
        throw std::runtime_error{"lack of param"};
    }
};

template <class T>
struct LoggerWrap : NodeMixin {
    explicit LoggerWrap(T& node)
        : node{node} {}

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

private:
    T& node;
};

} // namespace rmcs::navigation
