#pragma once
#include "cxx/util/pimpl.hh"

#include <limits>
#include <rclcpp/node.hpp>
#include <string>

#include <Eigen/Geometry>
#include <rmcs_msgs/chassis_mode.hpp>

namespace rmcs::navigation {

class MotionFsm {
    RMCS_PIMPL_DEFINITION(MotionFsm)

public:
    using ChassisMode = rmcs_msgs::ChassisMode;

    static constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNan = Eigen::Vector2d{kNan, kNan};

    struct Context {
        Eigen::Vector2d target_chassis_speed = kVecNan;
        Eigen::Vector2d target_gimbal_toward = kVecNan;

        double current_local_yaw = kNan;
        double current_world_yaw = kNan;

        double x = kNan, y = kNan;

        bool under_attack = false;
    } context;

    struct Command {
        ChassisMode chassis_mode = ChassisMode::ALIGNMENT;
        Eigen::Vector2d chassis_speed = kVecNan;
        Eigen::Vector2d gimbal_toward = kVecNan;
    } command;

    explicit MotionFsm(rclcpp::Node& node) noexcept;
    auto switch_mode(const std::string& mode) -> void;
    auto spin_once() noexcept -> Command;
};

} // namespace rmcs::navigation
