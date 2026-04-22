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
    bool lua_gimbal_direction_override_active_ = false;
    Eigen::Vector2d lua_gimbal_direction_override_ = Eigen::Vector2d{kNan_, kNan_};
    double last_navigation_goal_yaw_ = kNan_;
    const double gimbal_goal_deadzone_ = get_parameter_or<double>("gimbal_goal_deadzone", 0.15);
    const bool require_goal_heading_alignment_ =
        get_parameter_or<bool>("require_goal_heading_alignment", true);
    const double navigation_goal_yaw_tolerance_ =
        get_parameter_or<double>("navigation_goal_yaw_tolerance", 0.1);

    struct Command {
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_direction;
        OutputInterface<std::size_t> nod_count;
        OutputInterface<bool> rotate_chassis;
        OutputInterface<bool> detect_targets;
        OutputInterface<bool> enable_autoaim;

        auto init(Navigation& component) -> void {
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_direction", gimbal_direction,
                Eigen::Vector2d{
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                });
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
            "update_gimbal_direction", [this](double gimbal_angle, double pitch_angle) {
                set_lua_gimbal_direction_override(gimbal_angle, pitch_angle);
            });
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

    auto set_lua_gimbal_direction_override(double gimbal_angle, double pitch_angle) -> void {
        if (std::isfinite(gimbal_angle) && std::isfinite(pitch_angle)) {
            lua_gimbal_direction_override_active_ = true;
            lua_gimbal_direction_override_ = Eigen::Vector2d{gimbal_angle, pitch_angle};
            return;
        }

        lua_gimbal_direction_override_active_ = false;
        lua_gimbal_direction_override_ = Eigen::Vector2d{kNan_, kNan_};
    }

    static auto wrap_angle(double angle) -> double {
        constexpr auto kPi = std::numbers::pi_v<double>;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle <= -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    auto navigation_heading_aligned(double desired_yaw, double base_yaw) const -> bool {
        if (!require_goal_heading_alignment_)
            return true;
        if (!std::isfinite(desired_yaw))
            return false;
        if (!std::isfinite(base_yaw))
            return false;

        const auto yaw_error = wrap_angle(desired_yaw - base_yaw);
        return std::abs(yaw_error) <= std::max(navigation_goal_yaw_tolerance_, 0.0);
    }

    auto sync_gimbal_direction_to_navigation_goal() -> void {
        if (lua_gimbal_direction_override_active_) {
            *command.gimbal_direction = lua_gimbal_direction_override_;
            return;
        }

        const auto [goal_x, goal_y] = navigation.check_active_goal();
        const auto [x, y, base_yaw] = navigation.check_position();
        if (!std::isfinite(goal_x) || !std::isfinite(goal_y) || !std::isfinite(x)
            || !std::isfinite(y)) {
            if (std::isfinite(last_navigation_goal_yaw_)) {
                if (navigation_heading_aligned(last_navigation_goal_yaw_, base_yaw)) {
                    last_navigation_goal_yaw_ = kNan_;
                    *command.gimbal_direction = Eigen::Vector2d{kNan_, kNan_};
                    return;
                }
                *command.gimbal_direction = Eigen::Vector2d{last_navigation_goal_yaw_, 0.0};
                return;
            }

            *command.gimbal_direction = Eigen::Vector2d{kNan_, kNan_};
            return;
        }

        const auto dx = goal_x - x;
        const auto dy = goal_y - y;
        if (std::hypot(dx, dy) <= std::max(gimbal_goal_deadzone_, 0.0)) {
            if (!std::isfinite(last_navigation_goal_yaw_)) {
                *command.gimbal_direction = Eigen::Vector2d{kNan_, kNan_};
                return;
            }

            if (navigation_heading_aligned(last_navigation_goal_yaw_, base_yaw)) {
                last_navigation_goal_yaw_ = kNan_;
                *command.gimbal_direction = Eigen::Vector2d{kNan_, kNan_};
                return;
            }
            *command.gimbal_direction = Eigen::Vector2d{last_navigation_goal_yaw_, 0.0};
            return;
        }

        // Publish world-frame yaw target; gimbal controller closes the loop with encoder feedback.
        last_navigation_goal_yaw_ = std::atan2(dy, dx);
        *command.gimbal_direction = Eigen::Vector2d{last_navigation_goal_yaw_, 0.0};
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), option()}
        , context{*this, *this}
        , navigation{*this} {

        mock_context = param<bool>("mock_context");

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

        sync_gimbal_direction_to_navigation_goal();
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
