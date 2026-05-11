#pragma once

#include <format>
#include <utility>

#include <rclcpp/logging.hpp>

namespace rmcs::navigation {

/// @brief 基于 C++23 deduced this 的日志与参数混入
///
/// 继承此结构的派生类须提供 get_logger() 方法（通常来自 rclcpp::Node）。
/// 通过 using logging = NodeMixin 使派生类内可用 logging::info(...) 形式调用日志，
/// 编译器自动将派生类实例作为隐式对象参数传入，无额外运行时开销。
struct NodeMixin {
    using logging = NodeMixin;

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

/// @brief 将任意拥有 get_logger() 的对象适配为 NodeMixin
///
/// 用于非 NodeMixin 派生类的日志能力注入，通过组合方式持有目标对象引用并委托 get_logger()。
template <class T>
struct NodeWrap : NodeMixin {
    explicit NodeWrap(T& node)
        : node{node} {}

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

private:
    T& node;
};

} // namespace rmcs::navigation
