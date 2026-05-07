#include <string>
#if defined(__clang__)
# pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include "cxx/context.hh"
#include "cxx/util/navigation/navigation.hh"
#include "cxx/util/node_mixin.hh"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>

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

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_speed;
        OutputInterface<bool> sentry_decision_enabled;
        OutputInterface<std::uint16_t> sentry_bullet_exchange_value;
        OutputInterface<std::uint8_t> requested_mode;
        OutputInterface<bool> sentry_confirm_revive;
        std::chrono::steady_clock::time_point sentry_decision_clear_at =
            std::chrono::steady_clock::time_point::min();

        auto init(Navigation& component) -> void {
            component.register_output("/rmcs_navigation/enable_control", enable_control, false);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, false);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_velocity", gimbal_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/referee/sentry/decision/enabled", sentry_decision_enabled, false);
            component.register_output(
                "/referee/sentry/decision/bullet_exchange_value", sentry_bullet_exchange_value,
                std::uint16_t{0});
            component.register_output(
                "/referee/sentry/decision/mode", requested_mode, std::uint8_t{0});
            component.register_output(
                "/referee/sentry/decision/confirm_revive", sentry_confirm_revive, false);
        }

        auto exchange_17mm_bullet(int amount) -> void {
            constexpr int max_bullet_exchange_value = 0x07ff;
            amount = std::clamp(amount, 0, max_bullet_exchange_value);

            if (amount == 0) {
                clear_sentry_decision();
                return;
            }

            reset_all_decisions();
            *sentry_bullet_exchange_value = static_cast<std::uint16_t>(amount);
            activate_decision();
        }

        auto switch_mode(int mode) -> void {
            if (mode < 1 || mode > 3)
                return;

            reset_all_decisions();
            *requested_mode = static_cast<std::uint8_t>(mode);
            activate_decision();
        }

        auto confirm_revive() -> void {
            reset_all_decisions();
            *sentry_confirm_revive = true;
            activate_decision();
        }

        auto update() -> void {
            if (sentry_decision_clear_at == std::chrono::steady_clock::time_point::min())
                return;

            if (std::chrono::steady_clock::now() >= sentry_decision_clear_at)
                clear_sentry_decision();
        }

    private:
        auto reset_all_decisions() -> void {
            *sentry_bullet_exchange_value = std::uint16_t{0};
            *requested_mode = std::uint8_t{0};
            *sentry_confirm_revive = false;
        }

        auto activate_decision() -> void {
            *sentry_decision_enabled = true;
            sentry_decision_clear_at =
                std::chrono::steady_clock::now() + std::chrono::milliseconds{800};
        }

        auto clear_sentry_decision() -> void {
            *sentry_decision_enabled = false;
            reset_all_decisions();
            sentry_decision_clear_at = std::chrono::steady_clock::time_point::min();
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
            logging::fuck("\n{}", error.what());
            throw std::runtime_error(std::string{message});
        }
        return result;
    }

    auto make_api_injection() {
        auto api_result = unwrap_sol(
            lua->safe_script("return require('api')", sol::script_pass_on_error),
            "failed to get lua api");

        auto api = api_result.get<sol::table>();
        api.set_function(
            "info", [this](const std::string& text) { logging::info("Lua: {}", text); });
        api.set_function(
            "warn", [this](const std::string& text) { logging::warn("Lua: {}", text); });
        api.set_function(
            "fuck", [this](const std::string& text) { logging::fuck("Lua: {}", text); });

        // @TODO:
        //  补全这些实现
        api.set_function(
            "update_enable_control", [this](bool enable) { *command.enable_control = enable; });
        api.set_function(
            "send_target", [this](double x, double y) { navigation.send_target(x, y); });
        api.set_function("switch_topic_forward", [this](bool enable) {
            navigation.switch_topic_forward(enable);
        });
        api.set_function("update_gimbal_direction", [this](double angle) {
            logging::warn("unimplement: update_gimbal_direction({})", angle);
        });
        api.set_function("update_chassis_mode", [this](const std::string& mode) {
            logging::warn("unimplement: update_chassis_mode(\"{}\")", mode);
        });
        api.set_function("update_chassis_vel", [this](double x, double y) {
            *command.chassis_speed = Eigen::Vector2d{x, y};
        });
        api.set_function("exchange_17mm_bullet", [this](int amount) {
            command.exchange_17mm_bullet(amount);
        });
        api.set_function("switch_mode", [this](int mode) {
            command.switch_mode(mode);
        });
        api.set_function("confirm_revive", [this]()-> void {
            command.confirm_revive();
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

        logging::info("injected {} ros parameters into lua option", parameters.size());
    }

    auto lua_sync() {
        const auto [x, y, yaw] = navigation.check_position();

        auto read_context =
            []<typename T>(const details::Context::InputInterface<T>& input, T fallback) -> T {
            if (input.ready())
                return *input;

            return fallback;
        };

        auto user = lua_blackboard["user"].get<sol::table>();
        user["health"] = read_context(context.robot_health, std::uint16_t{0});
        user["bullet"] = read_context(context.robot_bullet, std::uint16_t{0});
        user["chassis_power_limit"] = read_context(context.chassis_power_limit_referee, 0.0);
        user["x"] = x;
        user["y"] = y;
        user["yaw"] = yaw;
        user["chassis_power"] = read_context(context.chassis_power_referee, 0.0);
        user["chassis_buffer_energy"] = read_context(context.chassis_buffer_energy_referee, 0.0);
        user["chassis_output_status"] = read_context(context.chassis_output_status, false);
        user["shooter_cooling"] = read_context(context.robot_shooter_cooling, std::int64_t{0});
        user["shooter_heat_limit"] =
            read_context(context.robot_shooter_heat_limit, std::int64_t{0});
        user["bullet_42mm"] = read_context(context.robot_42mm_bullet, std::uint16_t{0});
        user["fortress_17mm_bullet"] =
            read_context(context.robot_fortress_17mm_bullet, std::uint16_t{0});
        user["initial_speed"] = read_context(context.robot_initial_speed, 0.0F);
        user["shoot_timestamp"] = read_context(context.robot_shoot_timestamp, 0.0);

        auto game = lua_blackboard["game"].get<sol::table>();
        game["stage"] =
            rmcs_msgs::to_string(read_context(context.game_stage, rmcs_msgs::GameStage::UNKNOWN));
        game["sync_timestamp"] = read_context(context.sync_timestamp, std::uint64_t{0});
        game["outpost_health"] = read_context(context.ally_outpost_hp, std::uint16_t{0});
        game["base_health"] = read_context(context.ally_base_hp, std::uint16_t{0});
        game["hero_health"] = read_context(context.ally_hero_hp, std::uint16_t{0});
        game["infantry_1_health"] = read_context(context.ally_infantry_1_hp, std::uint16_t{0});
        game["infantry_2_health"] = read_context(context.ally_infantry_2_hp, std::uint16_t{0});
        game["engineer_health"] = read_context(context.ally_engineer_hp, std::uint16_t{0});
        game["remaining_time"] = read_context(context.stage_remain_time, std::uint16_t{0});
        game["gold_coin"] = read_context(context.remaining_gold_coin, std::uint16_t{0});
        game["exchangeable_ammunition_quantity"] =
            read_context(context.sentry_exchangeable_bullet, std::uint16_t{0});
        const auto our_dart_number_of_hits = static_cast<int>(
            read_context(context.dart_latest_hit_target_total_count, std::uint8_t{0}));
        game["our_dart_number_of_hits"] = our_dart_number_of_hits;
        game["our_dart_nmber_of_hits"] = our_dart_number_of_hits;
        game["fortress_occupied"] =
            read_context(context.ally_fortress_occupation_status, std::uint8_t{0}) != 0;
        game["big_energy_mechanism_activated"] =
            read_context(context.ally_big_energy_activation_status, std::uint8_t{0}) != 0;
        game["small_energy_mechanism_activated"] =
            read_context(context.ally_small_energy_activation_status, std::uint8_t{0}) != 0;
        game["red_score"] = read_context(context.red_score, std::uint32_t{0});
        game["blue_score"] = read_context(context.blue_score, std::uint32_t{0});

        auto set_position = [](sol::table position, double x, double y) {
            position["x"] = x;
            position["y"] = y;
        };
        set_position(
            game["hero_position"].get<sol::table>(),
            read_context(context.ally_hero_position_x, 0.0),
            read_context(context.ally_hero_position_y, 0.0));
        set_position(
            game["infantry_1_position"].get<sol::table>(),
            read_context(context.ally_infantry_1_position_x, 0.0),
            read_context(context.ally_infantry_1_position_y, 0.0));
        set_position(
            game["infantry_2_position"].get<sol::table>(),
            read_context(context.ally_infantry_2_position_x, 0.0),
            read_context(context.ally_infantry_2_position_y, 0.0));
        set_position(
            game["engineer_position"].get<sol::table>(),
            read_context(context.ally_engineer_position_x, 0.0),
            read_context(context.ally_engineer_position_y, 0.0));

        game["robot_id"] = static_cast<int>(
            read_context(context.robot_id, rmcs_msgs::RobotId{rmcs_msgs::RobotId::UNKNOWN}));
        game["can_confirm_free_revive"] =
            read_context(context.sentry_can_confirm_free_revive, false);
        game["can_exchange_instant_revive"] =
            read_context(context.sentry_can_exchange_instant_revive, false);
        game["instant_revive_cost"] =
            read_context(context.sentry_instant_revive_cost, std::uint16_t{0});
        game["exchanged_bullet"] =
            read_context(context.sentry_exchanged_bullet, std::uint16_t{0});
        game["remote_bullet_exchange_count"] =
            read_context(context.sentry_remote_bullet_exchange_count, std::uint8_t{0});
        game["sentry_mode"] = read_context(context.sentry_mode, std::uint8_t{0});
        game["energy_mechanism_activatable"] =
            read_context(context.sentry_energy_mechanism_activatable, false);

        auto play = lua_blackboard["play"].get<sol::table>();
        play["rswitch"] =
            rmcs_msgs::to_string(read_context(context.switch_right, rmcs_msgs::Switch::UNKNOWN));
        play["lswitch"] =
            rmcs_msgs::to_string(read_context(context.switch_left, rmcs_msgs::Switch::UNKNOWN));
        auto map_command = lua_blackboard["map_command"].get<sol::table>();
        map_command["x"] = read_context(context.map_command_event_x, 0.0);
        map_command["y"] = read_context(context.map_command_event_y, 0.0);
        map_command["keyboard"] =
            static_cast<int>(read_context(context.map_command_event_keyboard, std::uint8_t{0}));
        map_command["target_robot_id"] = static_cast<int>(
            read_context(context.map_command_event_target_robot_id, std::uint8_t{0}));
        map_command["source"] =
            static_cast<int>(read_context(context.map_command_event_source, std::uint16_t{0}));
        map_command["sequence"] =
            read_context(context.map_command_event_sequence, std::uint64_t{0});

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
            logging::warn("lua endpoint does not define optional on_exit(), fallback to no-op");
        }
        lua_on_control = (*lua)["on_control"];
        if (lua_on_control == sol::lua_nil) {
            lua_on_control =
                lua->safe_script("return function(_, _, _) end", sol::script_pass_on_error);
            logging::warn("lua endpoint does not define optional on_control(), fallback to no-op");
        }

        // Init Lua First
        auto init_result = unwrap_sol(lua_on_init(), "lua on_init failed");

        logging::info("Lua resource is loaded successfully");
    }

    auto lua_tick() { auto result = unwrap_sol(lua_on_tick(), "lua on_tick failed"); }

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

        logging::info("Navigation is initialized");
    }

    auto before_updating() -> void override {
        if (auto ok = context.health(); !ok) {
            logging::fuck("{}", ok.error());
            throw std::runtime_error{"Context Error"};
        }
    }

    auto update() -> void override {
        command.update();

        if (lua_tick_count++ == 100) {
            lua_tick_count = 0;
            auto lock = std::scoped_lock{io_mutex};
            lua_sync();
            lua_tick();
        }
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
