#pragma once

#include "component/util/pimpl.hh"

#include <expected>
#include <functional>
#include <rmcs_msgs/game_stage.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs_navigation {

struct PlanBox final {
    RMCS_PIMPL_DEFINITION(PlanBox)

public:
    struct Information {
        rmcs_msgs::GameStage game_stage = rmcs_msgs::GameStage::UNKNOWN;

        double current_x = 0;
        double current_y = 0;

        std::uint16_t health = 0;
        std::uint16_t bullet = 0;
    };

    auto configure(const YAML::Node&) -> std::expected<void, std::string>;

    auto set_printer(std::function<void(const std::string&)>) -> void;

    auto set_config_name(const std::string&) -> void;

    template <std::invocable<Information&> F>
    auto update_information(F&& function) noexcept -> void {
        std::forward<F>(function)(information_());
        do_plan_();
    }

    auto goal_position() noexcept -> std::tuple<double, double>;

    auto rotate_chassis() const noexcept -> bool;

    auto gimbal_scanning() const noexcept -> bool;

private:
    auto do_plan_() noexcept -> void;
    auto information_() noexcept -> Information&;
};

} // namespace rmcs_navigation
