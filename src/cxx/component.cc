#include "cxx/context.hh"
#include "cxx/controller/motion.hh"
#include "cxx/lua_context.hh"
#include "cxx/navigation.hh"
#include "cxx/util/node_mixin.hh"

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/rmcs_msgs.hpp>

#include <string>
#include <unordered_map>

namespace rmcs::navigation {

class Navigation final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::NodeMixin {
private:
    static constexpr auto kCmdVelTimeout = std::chrono::milliseconds{500};
    static constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNaN = Eigen::Vector2d{kNan, kNan};

    std::atomic<std::uint16_t> lua_tick_count = 0;
    double pending_climb_world_yaw = kNan;

    details::LuaContext lua{*this};
    details::Navigation nav{*this};
    details::RmcsContext rmcs{*this};

    MotionFsm motion{*this};

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;
        using SentryEventCounts = std::unordered_map<rmcs_msgs::SentryEvent, std::uint16_t>;

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<bool> enable_supercap_;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_toward;
        OutputInterface<double> climb_cross_direction;
        OutputInterface<bool> climb_is_climb;
        OutputInterface<bool> track_rune;
        OutputInterface<bool> automatic_resurrection;
        OutputInterface<SentryEventCounts> sentry_events;

        explicit Command(Navigation& component) {
            component.register_output("/rmcs_navigation/enable_control", enable_control, true);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, true);
            component.register_output("/rmcs_navigation/enable_supercap", enable_supercap_, false);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output("/rmcs_navigation/chassis_velocity", chassis_speed, kVecNaN);
            component.register_output("/rmcs_navigation/gimbal_toward", gimbal_toward, kVecNaN);
            component.register_output(
                "/rmcs_navigation/request/cross_direction", climb_cross_direction, kNan);
            component.register_output("/rmcs_navigation/request/is_climb", climb_is_climb, false);
            component.register_output("/rmcs_navigation/request/track_rune", track_rune, false);
            component.register_output(
                "/rmcs_navigation/automatic_resurrection", automatic_resurrection, true);
            component.register_output(
                "/rmcs_navigation/sentry_events", sentry_events, SentryEventCounts{});
        }
    } command{*this};

    auto sync_blackboard() {
        auto [x, y, yaw] = nav.check_position();

        // 高频查询 TF 是不对的，所以应该先缓存一份
        motion.context.current_world_yaw = yaw;
        ++motion.context.yaw_sample;
        motion.context.x = x;
        motion.context.y = y;

        auto& blackboard = lua.blackboard();

        auto user = blackboard["user"].get<sol::table>();
        user["health"] = *rmcs.robot_health;
        user["bullet"] = *rmcs.robot_bullet;
        user["chassis_power_limit"] = *rmcs.chassis_power_limit_referee;
        user["x"] = x;
        user["y"] = y;
        user["yaw"] = yaw;

        auto game = blackboard["game"].get<sol::table>();
        game["stage"] = rmcs_msgs::to_string(*rmcs.game_stage);
        game["enemy_outpost_hp"] = *rmcs.enemy_outpost_hp;
        game["enemy_base_hp"] = *rmcs.enemy_base_hp;

        auto play = blackboard["play"].get<sol::table>();
        play["rswitch"] = rmcs_msgs::to_string(*rmcs.switch_right);
        play["lswitch"] = rmcs_msgs::to_string(*rmcs.switch_left);

        auto autoaim = blackboard["autoaim"].get<sol::table>();
        autoaim["should_control"] = *rmcs.auto_aim_should_control;

        auto map_command = blackboard["map_command"].get<sol::table>();
        map_command["x"] = *rmcs.map_command_x;
        map_command["y"] = *rmcs.map_command_y;

        auto energy = blackboard["energy"].get<sol::table>();
        energy["small"] = *rmcs.ally_small_energy_core_state;
        energy["big"] = *rmcs.ally_big_energy_core_state;

        auto meta = blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), node::option()} {

        rmcs.init();

        lua.inject(
            "update_enable_control", [this](bool enable) { *command.enable_control = enable; });
        lua.inject("send_target", [this](double x, double y) { nav.send_target(x, y); });
        lua.inject("cancel_target", [this] { nav.cancel_target(); });
        lua.inject(
            "switch_topic_forward", [this](bool enable) { nav.switch_topic_forward(enable); });
        lua.inject("update_gimbal_direction", [this](double yaw, double pitch) {
            motion.context.target_gimbal_toward = {yaw, pitch};
        });
        lua.inject(
            "switch_motion_mode", [this](const std::string& mode) { motion.switch_mode(mode); });
        lua.inject("update_under_attack", [this](bool yes) { motion.context.under_attack = yes; });
        lua.inject(
            "update_supercap_boost", [this](bool enable) { *command.enable_supercap_ = enable; });
        lua.inject("update_track_rune", [this](bool enable) { *command.track_rune = enable; });
        lua.inject("set_automatic_resurrection", [this](bool enable) {
            *command.automatic_resurrection = enable;
        });

        lua.inject("set_climb_direction", [this](double world_yaw) {
            if (std::isfinite(world_yaw)) {
                pending_climb_world_yaw = world_yaw;
            } else {
                pending_climb_world_yaw = kNan;
                *command.climb_cross_direction = kNan;
            }
        });
        lua.inject(
            "set_climb_switch", [this](bool is_climb) { *command.climb_is_climb = is_climb; });
        lua.inject("get_climb_status", [this] { return *rmcs.climber_status; });

        lua.inject("relocalize", [this] { nav.relocalize(rmcs.robot_id->color()); });

        lua.inject("sentry_event", [this](const std::string& name) {
            using SentryEvent = rmcs_msgs::SentryEvent;
            static const auto table = std::unordered_map<std::string, SentryEvent>{
                {"SWITCH_POSE_ATTACK", SentryEvent::SWITCH_POSE_ATTACK},
                {"SWITCH_POSE_DEFENSE", SentryEvent::SWITCH_POSE_DEFENSE},
                {"SWITCH_POSE_MOVE", SentryEvent::SWITCH_POSE_MOVE},
                {"SWITCH_POSE_POWERED_ATTACK", SentryEvent::SWITCH_POSE_POWERED_ATTACK},
                {"SWITCH_POSE_POWERED_DEFENSE", SentryEvent::SWITCH_POSE_POWERED_DEFENSE},
                {"SWITCH_POSE_POWERED_MOVE", SentryEvent::SWITCH_POSE_POWERED_MOVE},
                {"CONFIRM_REBIRTH", SentryEvent::CONFIRM_REBIRTH},
                {"CONFIRM_INSTANT_REBIRTH", SentryEvent::CONFIRM_INSTANT_REBIRTH},
                {"EXCHANGE_AMMO_SUPPLY_POINT", SentryEvent::EXCHANGE_AMMO_SUPPLY_POINT},
                {"EXCHANGE_AMMO_REMOTE", SentryEvent::EXCHANGE_AMMO_REMOTE},
                {"EXCHANGE_HP_REMOTE", SentryEvent::EXCHANGE_HP_REMOTE},
                {"ACTIVATE_ENERGY_CORE", SentryEvent::ACTIVATE_ENERGY_CORE},
            };

            if (const auto it = table.find(name); it != table.end()) {
                ++(*command.sentry_events)[it->second];
            } else {
                node::warn("Unknown sentry event: {}", name);
            }
        });

        node::info("Navigation is initialized");
    }

    auto before_updating() -> void override { rmcs.ensure_defaults(); }

    auto update() -> void override {
        const auto gimbal_direction = fast_tf::cast<rmcs_description::OdomGimbalImu>(
            rmcs_description::BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *rmcs.tf);
        motion.context.current_gimbal_yaw =
            std::atan2(gimbal_direction->y(), gimbal_direction->x());

        if (lua_tick_count++ == 10) [[unlikely]] {
            lua_tick_count = 0;
            sync_blackboard();
            lua.tick();
        }

        {
            *command.chassis_speed = Eigen::Vector2d::Zero();
            *command.gimbal_toward = Eigen::Vector2d::Zero();
            *command.chassis_behavior = rmcs_msgs::ChassisMode::SPIN_FAST;
        }

        const auto nav_cmd = nav.current_command();
        const auto elapsed = std::chrono::steady_clock::now() - nav_cmd.timestamp;
        motion.context.target_chassis_speed =
            (elapsed > kCmdVelTimeout) ? Eigen::Vector2d::Zero() : nav_cmd.speed;

        const auto cmd = motion.spin_once();
        if (std::isfinite(pending_climb_world_yaw)) {
            const auto target = motion.world2gimbal_odom(pending_climb_world_yaw);
            if (std::isfinite(target)) {
                *command.climb_cross_direction = target;
                pending_climb_world_yaw = kNan;
            }
        }
        if (*command.enable_control) {
            *command.chassis_speed = cmd.chassis_speed;
            *command.gimbal_toward = cmd.gimbal_toward;
            *command.chassis_behavior = cmd.chassis_mode;
        }
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
