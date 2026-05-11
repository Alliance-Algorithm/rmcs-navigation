#pragma once
#include "util/pimpl.hh"

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/string.hpp>

#include <expected>
#include <mutex>
#include <string>

namespace rmcs::navigation::details {

struct Context {
    RMCS_PIMPL_DEFINITION(Context)
public:
    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;
    InputInterface<std::uint32_t> red_score;
    InputInterface<std::uint32_t> blue_score;
    InputInterface<rmcs_msgs::Switch> switch_right;
    InputInterface<rmcs_msgs::Switch> switch_left;
    InputInterface<double> chassis_power_limit_referee;
    InputInterface<rmcs_description::SentryTf> tf;
    InputInterface<Eigen::Vector3d> enemy_center;

    explicit Context(rclcpp::Node& node, rmcs_executor::Component& component) noexcept;

    auto init(std::mutex& io_mutex, bool mock = false) -> void;
    auto from(const std::string& raw) noexcept -> std::expected<void, std::string>;

    auto health() const noexcept -> std::expected<void, std::string>;
};

} // namespace rmcs::navigation::details
