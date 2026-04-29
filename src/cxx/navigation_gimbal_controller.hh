#pragma once
#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <mutex>
#include <optional>

namespace rmcs::navigation::details {

class Navigation;

class NavigationGimbalController {
public:
    using Output = rmcs_executor::Component::OutputInterface<Eigen::Vector2d>;

    NavigationGimbalController(rclcpp::Node& node, Navigation& navigation);

    auto init_output(rmcs_executor::Component& component) -> void;
    auto init_params() -> void;
    auto set_lua_target_yaw(double yaw) -> void;
    auto update() -> void;

private:
    auto get_lua_target_yaw() const -> std::optional<double>;
    auto smooth_yaw_speed(double target_speed) -> double;
    auto sync_to_navigation_goal() -> void;
    auto update_test_pattern() -> void;

    rclcpp::Node& node_;
    Navigation& navigation_;

    Output gimbal_speed_;

    mutable std::mutex lua_mutex_;
    double lua_target_yaw_ = std::numeric_limits<double>::quiet_NaN();
    bool lua_target_yaw_active_ = false;

    double last_navigation_goal_yaw_ = std::numeric_limits<double>::quiet_NaN();

    double kp_ = 0.5;
    double speed_max_ = 0.8;
    double tolerance_ = std::numbers::pi_v<double> / 18.0;
    double smooth_alpha_ = 0.02;
    double acc_limit_ = 2.0;
    double speed_command_ = 0.0;

    bool test_mode_ = false;
    double test_yaw_speed_ = 2.0;
    double test_pitch_amplitude_ = 0.3;
    double test_pitch_frequency_ = 0.2;
};

} // namespace rmcs::navigation::details
