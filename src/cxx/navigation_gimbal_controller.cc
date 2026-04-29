#include "navigation_gimbal_controller.hh"
#include "navigation.hh"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs::navigation::details {

NavigationGimbalController::NavigationGimbalController(rclcpp::Node& node, Navigation& navigation)
    : node_{node}
    , navigation_{navigation} {}

auto NavigationGimbalController::init_output(rmcs_executor::Component& component) -> void {
    component.register_output(
        "/rmcs_navigation/gimbal_velocity", gimbal_speed_, Eigen::Vector2d::Zero());
}

auto NavigationGimbalController::init_params() -> void {
    kp_ = node_.get_parameter_or<double>("navigation_gimbal_yaw_kp", 0.2);
    speed_max_ = node_.get_parameter_or<double>("navigation_gimbal_yaw_speed_max", 0.5);
    tolerance_ = node_.get_parameter_or<double>(
        "navigation_gimbal_yaw_tolerance",
        node_.get_parameter_or<double>("navigation_gimbal_yaw_tolerance_deg", 8.0)
            * (std::numbers::pi_v<double> / 180.0));
    smooth_alpha_ = node_.get_parameter_or<double>("navigation_gimbal_yaw_smooth_alpha", 0.02);
    acc_limit_ = node_.get_parameter_or<double>("navigation_gimbal_yaw_acc_limit", 1.0);
    test_mode_ = node_.get_parameter_or<bool>("navigation_gimbal_velocity_test_mode", false);
    test_yaw_speed_ =
        node_.get_parameter_or<double>("navigation_gimbal_velocity_test_yaw_speed", 0.2);
    test_pitch_amplitude_ = node_.get_parameter_or<double>(
        "navigation_gimbal_velocity_test_pitch_speed_amplitude", 0.3);
    test_pitch_frequency_ = node_.get_parameter_or<double>(
        "navigation_gimbal_velocity_test_pitch_speed_frequency", 0.2);
}

auto NavigationGimbalController::set_lua_target_yaw(double yaw) -> void {
    auto lock = std::scoped_lock{lua_mutex_};
    if (std::isfinite(yaw)) {
        lua_target_yaw_ = yaw;
        lua_target_yaw_active_ = true;
        return;
    }

    lua_target_yaw_ = std::numeric_limits<double>::quiet_NaN();
    lua_target_yaw_active_ = false;
}

auto NavigationGimbalController::get_lua_target_yaw() const -> std::optional<double> {
    auto lock = std::scoped_lock{lua_mutex_};
    if (!lua_target_yaw_active_ || !std::isfinite(lua_target_yaw_))
        return std::nullopt;

    return lua_target_yaw_;
}

auto NavigationGimbalController::smooth_yaw_speed(double target_speed) -> double {
    constexpr double kControlDt = 1e-3;

    target_speed = std::clamp(target_speed, -speed_max_, speed_max_);

    const auto alpha = std::clamp(smooth_alpha_, 0.0, 1.0);
    const auto prev = speed_command_;
    auto filtered = prev + alpha * (target_speed - prev);

    if (acc_limit_ > 0.0) {
        const auto max_step = acc_limit_ * kControlDt;
        const auto delta = std::clamp(filtered - prev, -max_step, max_step);
        filtered = prev + delta;
    }

    if (std::abs(filtered) < 1e-4)
        filtered = 0.0;

    speed_command_ = filtered;
    return filtered;
}

auto NavigationGimbalController::sync_to_navigation_goal() -> void {
    const auto current_yaw = std::get<2>(navigation_.check_position());

    auto apply_yaw_target = [&](double target_yaw) {
        auto target_yaw_speed = 0.0;
        if (std::isfinite(target_yaw) && std::isfinite(current_yaw)) {
            const auto yaw_error =
                std::remainder(target_yaw - current_yaw, 2.0 * std::numbers::pi_v<double>);
            if (std::abs(yaw_error) > tolerance_)
                target_yaw_speed = kp_ * yaw_error;
        }
        *gimbal_speed_ = Eigen::Vector2d{smooth_yaw_speed(target_yaw_speed), 0.0};
    };

    // Lua override always wins — does not pollute last_navigation_goal_yaw_
    if (auto override_yaw = get_lua_target_yaw(); override_yaw.has_value()) {
        apply_yaw_target(*override_yaw);
        return;
    }

    // Compute target yaw from the active navigation goal (goal-yaw > position-delta)
    constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

    const auto [goal_x, goal_y, goal_yaw] = navigation_.check_active_goal();
    const auto position = navigation_.check_position();
    const auto x = std::get<0>(position);
    const auto y = std::get<1>(position);

    auto nav_target_yaw = kNan;
    if (std::isfinite(goal_yaw))
        nav_target_yaw = goal_yaw;
    else if (std::isfinite(goal_x) && std::isfinite(goal_y) && std::isfinite(x) && std::isfinite(y))
        nav_target_yaw = std::atan2(goal_y - y, goal_x - x);

    // Persist valid navigation target yaw so the gimbal can continue turning even
    // after Nav2 considers the position reached and clears the active goal.
    if (std::isfinite(nav_target_yaw))
        last_navigation_goal_yaw_ = nav_target_yaw;

    auto effective_yaw = nav_target_yaw;
    if (!std::isfinite(effective_yaw))
        effective_yaw = last_navigation_goal_yaw_;

    apply_yaw_target(effective_yaw);

    // Once heading is aligned AND the navigation goal is gone, release the hold
    if (!std::isfinite(nav_target_yaw) && std::isfinite(last_navigation_goal_yaw_)
        && std::isfinite(current_yaw)) {
        const auto yaw_error = std::remainder(
            last_navigation_goal_yaw_ - current_yaw, 2.0 * std::numbers::pi_v<double>);
        if (std::abs(yaw_error) <= tolerance_)
            last_navigation_goal_yaw_ = kNan;
    }
}

auto NavigationGimbalController::update_test_pattern() -> void {
    const auto t = node_.now().seconds();
    const auto pitch_speed = test_pitch_amplitude_
                           * std::sin(2.0 * std::numbers::pi_v<double> * test_pitch_frequency_ * t);

    *gimbal_speed_ = Eigen::Vector2d{test_yaw_speed_, pitch_speed};
}

auto NavigationGimbalController::update() -> void {
    if (test_mode_) {
        update_test_pattern();
    } else {
        sync_to_navigation_goal();
    }
}

} // namespace rmcs::navigation::details
