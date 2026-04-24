#if defined(__clang__)
# pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "cxx/context.hh"
#include "cxx/navigation.hh"
#include "cxx/util/node_mixin.hh"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <tuple>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/rmcs_msgs.hpp> // IWYU pragma: keep
#include <sol/sol.hpp>

namespace rmcs::navigation {

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::NodeMixin {
private:
    mutable std::mutex io_mutex;
    static constexpr auto kNan_ = std::numeric_limits<double>::quiet_NaN();

    using Twist = geometry_msgs::msg::Twist;
    std::shared_ptr<rclcpp::Subscription<Twist>> subscription_twist;

    bool mock_context = false;

    std::atomic<std::uint16_t> lua_tick_count = 0;
    std::unique_ptr<sol::state> lua;
    sol::table lua_blackboard;
    sol::protected_function lua_on_init;
    sol::protected_function lua_on_tick;
    sol::protected_function lua_on_exit;
    sol::protected_function lua_on_control;

    details::Context context;
    details::Navigation navigation;
    mutable std::mutex lua_gimbal_target_yaw_mutex_;
    double lua_gimbal_target_yaw_ = kNan_;
    bool lua_gimbal_target_yaw_active_ = false;
    double navigation_gimbal_yaw_kp_ = 0.5;
    double navigation_gimbal_yaw_speed_max_ = 0.8;
    double navigation_gimbal_yaw_tolerance_ = std::numbers::pi_v<double> / 18.0;
    double navigation_gimbal_yaw_smooth_alpha_ = 0.02;
    double navigation_gimbal_yaw_acc_limit_ = 2.0;
    double navigation_gimbal_yaw_speed_command_ = 0.0;

    struct Command {
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_speed;
        OutputInterface<std::size_t> nod_count;
        OutputInterface<bool> rotate_chassis;
        OutputInterface<bool> detect_targets;
        OutputInterface<bool> enable_autoaim;

        auto init(Navigation& component) -> void {
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_velocity", gimbal_speed, Eigen::Vector2d::Zero());
            component.register_output("/rmcs_navigation/nod_count", nod_count, 0);
            component.register_output("/rmcs_navigation/rotate_chassis", rotate_chassis, false);
            component.register_output("/rmcs_navigation/detect_targets", detect_targets, false);
            component.register_output("/rmcs_navigation/start_autoaim", enable_autoaim, false);
        }
    } command;

private:
    static auto option() noexcept {
        return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    }

    template <typename T>
    auto unwrap_sol(T result, std::string_view message) -> T {
        if (!result.valid()) {
            auto error = result.template get<sol::error>();
            fuck("\n{}", error.what());
            throw std::runtime_error(std::string{message});
        }
        return result;
    }

    auto make_api_injection() {
        auto api_result = unwrap_sol(
            lua->safe_script("return require('api')", sol::script_pass_on_error),
            "failed to get lua api");

        auto api = api_result.get<sol::table>();
        api.set_function("info", [this](const std::string& text) { info("Lua: {}", text); });
        api.set_function("warn", [this](const std::string& text) { warn("Lua: {}", text); });
        api.set_function("fuck", [this](const std::string& text) { fuck("Lua: {}", text); });

        // @TODO:
        //  补全这些实现
        api.set_function(
            "send_target", [this](double x, double y) { navigation.send_target(x, y); });
        api.set_function("switch_topic_forward", [this](bool enable) {
            navigation.switch_topic_forward(enable);
        });
        api.set_function(
            "update_gimbal_direction", [this](double angle) { set_lua_gimbal_target_yaw(angle); });
        api.set_function("update_chassis_mode", [this](const std::string& mode) {
            warn("unimplement: update_chassis_mode(\"{}\")", mode);
        });
        api.set_function("update_chassis_vel", [this](double x, double y) {
            *command.chassis_speed = Eigen::Vector2d{x, y};
        });
    }

    auto make_option_injection() {
        auto option_result = unwrap_sol(
            lua->safe_script("return require('option')", sol::script_pass_on_error),
            "failed to get lua option");

        auto option = option_result.get<sol::table>();

        auto parameters = std::map<std::string, rclcpp::Parameter>{};
        get_parameters("", parameters);

        auto to_lua_array = [this]<typename T>(const std::vector<T>& values) {
            auto array = lua->create_table(static_cast<int>(values.size()), 0);
            for (auto index = std::size_t{0}; index < values.size(); ++index)
                array[index + 1] = values[index];

            return array;
        };

        for (const auto& [name, parameter] : parameters) {
            switch (parameter.get_type()) {
            case rclcpp::PARAMETER_BOOL: option[name] = parameter.as_bool(); break;
            case rclcpp::PARAMETER_INTEGER: option[name] = parameter.as_int(); break;
            case rclcpp::PARAMETER_DOUBLE: option[name] = parameter.as_double(); break;
            case rclcpp::PARAMETER_STRING: option[name] = parameter.as_string(); break;
            case rclcpp::PARAMETER_BOOL_ARRAY:
                option[name] = to_lua_array(parameter.as_bool_array());
                break;
            case rclcpp::PARAMETER_INTEGER_ARRAY:
                option[name] = to_lua_array(parameter.as_integer_array());
                break;
            case rclcpp::PARAMETER_DOUBLE_ARRAY:
                option[name] = to_lua_array(parameter.as_double_array());
                break;
            case rclcpp::PARAMETER_STRING_ARRAY:
                option[name] = to_lua_array(parameter.as_string_array());
                break;
            default: option[name] = sol::lua_nil; break;
            }
        }

        info("injected {} ros parameters into lua option", parameters.size());
    }

    auto lua_sync() {
        const auto [x, y, yaw] = navigation.check_position();

        auto user = lua_blackboard["user"].get<sol::table>();
        user["health"] = *context.robot_health;
        user["bullet"] = *context.robot_bullet;
        user["chassis_power_limit"] = *context.chassis_power_limit_referee;
        user["x"] = x;
        user["y"] = y;
        user["yaw"] = yaw;

        auto game = lua_blackboard["game"].get<sol::table>();
        game["stage"] = rmcs_msgs::to_string(*context.game_stage);

        auto play = lua_blackboard["play"].get<sol::table>();
        play["rswitch"] = rmcs_msgs::to_string(*context.switch_right);
        play["lswitch"] = rmcs_msgs::to_string(*context.switch_left);

        auto meta = lua_blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

    auto lua_init() {
        lua = std::make_unique<sol::state>();
        lua->open_libraries(
            sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::os, sol::lib::package,
            sol::lib::string, sol::lib::table, sol::lib::debug, sol::lib::io);

        // Load Lua Env Path
        auto package_root =
            std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs-navigation")};
        auto lua_root = package_root / "lua";
        auto package = (*lua)["package"].get<sol::table>();
        auto package_path = package["path"].get_or(std::string{});
        package["path"] = std::format(
            "{};{}/?.lua;{}/?/init.lua", package_path, lua_root.string(), lua_root.string());

        // Api Injection
        make_api_injection();
        make_option_injection();

        // Load Function Binding
        auto endpoint = param<std::string>("endpoint");
        auto required = std::format("require('endpoint.{}')", endpoint);
        auto load_result = unwrap_sol(
            lua->safe_script(required, sol::script_pass_on_error), "failed to load lua main");

        lua_blackboard = (*lua)["blackboard"];
        lua_on_init = (*lua)["on_init"];
        lua_on_tick = (*lua)["on_tick"];

        const auto situation = std::array{
            lua_on_init.valid(),
            lua_on_tick.valid(),
        };
        if (!std::ranges::all_of(situation, std::identity{}))
            throw std::runtime_error("lua main must define on_init() and on_tick()");

        lua_on_exit = (*lua)["on_exit"];
        if (lua_on_exit == sol::lua_nil) {
            lua_on_exit = lua->safe_script("return function() end", sol::script_pass_on_error);
            warn("lua endpoint does not define optional on_exit(), fallback to no-op");
        }
        lua_on_control = (*lua)["on_control"];
        if (lua_on_control == sol::lua_nil) {
            lua_on_control =
                lua->safe_script("return function(_, _, _) end", sol::script_pass_on_error);
            warn("lua endpoint does not define optional on_control(), fallback to no-op");
        }

        // Init Lua First
        auto init_result = unwrap_sol(lua_on_init(), "lua on_init failed");

        info("Lua resource is loaded successfully");
    }

    auto lua_tick() { auto result = unwrap_sol(lua_on_tick(), "lua on_tick failed"); }

    auto set_lua_gimbal_target_yaw(double yaw) -> void {
        auto lock = std::scoped_lock{lua_gimbal_target_yaw_mutex_};
        if (std::isfinite(yaw)) {
            lua_gimbal_target_yaw_ = yaw;
            lua_gimbal_target_yaw_active_ = true;
            return;
        }

        lua_gimbal_target_yaw_ = kNan_;
        lua_gimbal_target_yaw_active_ = false;
    }

    auto get_lua_gimbal_target_yaw() const -> std::optional<double> {
        auto lock = std::scoped_lock{lua_gimbal_target_yaw_mutex_};
        if (!lua_gimbal_target_yaw_active_ || !std::isfinite(lua_gimbal_target_yaw_))
            return std::nullopt;

        return lua_gimbal_target_yaw_;
    }

    auto calculate_navigation_target_yaw() const -> double {
        if (auto override_yaw = get_lua_gimbal_target_yaw(); override_yaw.has_value())
            return *override_yaw;

        const auto [goal_x, goal_y, goal_yaw] = navigation.check_active_goal();
        const auto position = navigation.check_position();
        const auto x = std::get<0>(position);
        const auto y = std::get<1>(position);

        if (std::isfinite(goal_yaw))
            return goal_yaw;

        if (std::isfinite(goal_x) && std::isfinite(goal_y) && std::isfinite(x) && std::isfinite(y))
            return std::atan2(goal_y - y, goal_x - x);

        return kNan_;
    }

    auto smooth_navigation_yaw_speed(double target_speed) -> double {
        constexpr double kControlDt = 1e-3;

        target_speed = std::clamp(
            target_speed, -navigation_gimbal_yaw_speed_max_, navigation_gimbal_yaw_speed_max_);

        const auto alpha = std::clamp(navigation_gimbal_yaw_smooth_alpha_, 0.0, 1.0);
        const auto prev = navigation_gimbal_yaw_speed_command_;
        auto filtered = prev + alpha * (target_speed - prev);

        if (navigation_gimbal_yaw_acc_limit_ > 0.0) {
            const auto max_step = navigation_gimbal_yaw_acc_limit_ * kControlDt;
            const auto delta = std::clamp(filtered - prev, -max_step, max_step);
            filtered = prev + delta;
        }

        if (std::abs(filtered) < 1e-4)
            filtered = 0.0;

        navigation_gimbal_yaw_speed_command_ = filtered;
        return filtered;
    }

    auto sync_gimbal_velocity_to_navigation_goal() -> void {
        const auto target_yaw = calculate_navigation_target_yaw();
        auto current_yaw = std::get<2>(navigation.check_position());
        auto target_yaw_speed = 0.0;

        if (std::isfinite(target_yaw) && std::isfinite(current_yaw)) {
            const auto yaw_error =
                std::remainder(target_yaw - current_yaw, 2.0 * std::numbers::pi_v<double>);
            if (std::abs(yaw_error) > navigation_gimbal_yaw_tolerance_) {
                target_yaw_speed = navigation_gimbal_yaw_kp_ * yaw_error;
            }
        }

        const auto yaw_speed = smooth_navigation_yaw_speed(target_yaw_speed);

        *command.gimbal_speed = Eigen::Vector2d{yaw_speed, 0.0};
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), option()}
        , context{*this, *this}
        , navigation{*this} {

        mock_context = param<bool>("mock_context");
        navigation_gimbal_yaw_kp_ = get_parameter_or<double>("navigation_gimbal_yaw_kp", 0.2);
        navigation_gimbal_yaw_speed_max_ =
            get_parameter_or<double>("navigation_gimbal_yaw_speed_max", 0.5);
        navigation_gimbal_yaw_tolerance_ = get_parameter_or<double>(
            "navigation_gimbal_yaw_tolerance",
            get_parameter_or<double>("navigation_gimbal_yaw_tolerance_deg", 8.0)
                * (std::numbers::pi_v<double> / 180.0));
        navigation_gimbal_yaw_smooth_alpha_ =
            get_parameter_or<double>("navigation_gimbal_yaw_smooth_alpha", 0.02);
        navigation_gimbal_yaw_acc_limit_ =
            get_parameter_or<double>("navigation_gimbal_yaw_acc_limit", 1.0);

        context.init(io_mutex, mock_context);
        command.init(*this);

        lua_init();

        const auto command_vel_name = param<std::string>("command_vel_name");
        subscription_twist = Node::create_subscription<Twist>(
            command_vel_name, 10, [this](const std::unique_ptr<Twist>& msg) {
                auto lock = std::scoped_lock{io_mutex};

                auto vx = msg->linear.x;
                auto vy = msg->linear.y;
                auto qx = msg->angular.x;
                unwrap_sol(lua_on_control(vx, vy, qx), "lua on_control failed");
            });

        info("Navigation is initialized");
    }

    auto update() -> void override {
        if (lua_tick_count++ == 100) {
            lua_tick_count = 0;
            auto lock = std::scoped_lock{io_mutex};
            lua_sync();
            lua_tick();
        }

        sync_gimbal_velocity_to_navigation_goal();
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
