#pragma once

#include <array>
#include <format>
#include <utility>

#include <rclcpp/logging.hpp>

namespace rmcs::navigation {

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
    auto fuck(this const Self& self, std::format_string<Args...> fmt, Args&&... args) -> void {
        auto text = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(self.get_logger(), "%s", text.c_str());
    }

    static constexpr auto kAsciiIcon = std::array{
        "|       ___           ___           ___           ___      |",
        "|      ╱╲  ╲         ╱╲__╲         ╱╲  ╲         ╱╲  ╲     |",
        "|     ╱::╲  ╲       ╱::│  │       ╱::╲  ╲       ╱::╲  ╲    |",
        "|    ╱:╱╲:╲  ╲     ╱:│:│  │      ╱:╱╲:╲  ╲     ╱:╱╲ ╲  ╲   |",
        "|   ╱::╲~╲:╲  ╲   ╱:╱│:│__│__   ╱:╱  ╲:╲  ╲   _╲:╲~╲ ╲  ╲  |",
        "|  ╱:╱╲:╲ ╲:╲__╲ ╱:╱ │::::╲__╲ ╱:╱__╱ ╲:╲__╲ ╱╲ ╲:╲ ╲ ╲__╲ |",
        "|  ╲╱_│::╲╱:╱  ╱ ╲╱__╱~~╱:╱  ╱ ╲:╲  ╲  ╲╱__╱ ╲:╲ ╲:╲ ╲╱__╱ |",
        "|     │:│::╱  ╱        ╱:╱  ╱   ╲:╲  ╲        ╲:╲ ╲:╲__╲   |",
        "|     │:│╲╱__╱        ╱:╱  ╱     ╲:╲  ╲        ╲:╲╱:╱  ╱   |",
        "|     │:│  │         ╱:╱  ╱       ╲:╲__╲        ╲::╱  ╱    |",
        "|      ╲│__│         ╲╱__╱         ╲╱__╱         ╲╱__╱     |",
    };
    auto print_icon(this const auto& self) {
        for (const auto* line : kAsciiIcon)
            self.info("{}", line);
    }
};

} // namespace rmcs::navigation
