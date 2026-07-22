#pragma once
#include "util/pimpl.hh"

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs::navigation::details {

struct RmcsContext {
    RMCS_PIMPL_DEFINITION(RmcsContext)
public:
    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;

    InputInterface<rmcs_msgs::Switch> switch_right;
    InputInterface<rmcs_msgs::Switch> switch_left;
    InputInterface<Eigen::Vector2d> rjoystick;
    InputInterface<Eigen::Vector2d> ljoystick;

    InputInterface<double> chassis_power_limit_referee;
    InputInterface<bool> auto_aim_should_control;

    InputInterface<bool> should_shoot;

    InputInterface<rmcs_description::SentryTf> tf;
    InputInterface<Eigen::Vector3d> enemy_center;

    explicit RmcsContext(auto& node_and_component) noexcept
        : RmcsContext{node_and_component, node_and_component} {}

    explicit RmcsContext(rclcpp::Node& node, rmcs_executor::Component& component) noexcept;

    auto init() -> void;
    auto ensure_defaults() -> void;
};

} // namespace rmcs::navigation::details
