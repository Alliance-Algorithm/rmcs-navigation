#include "cxx/context.hh"
#include "cxx/controller/motion.hh"
#include "cxx/lua_context.hh"
#include "cxx/navigation.hh"
#include "cxx/util/node_mixin.hh"

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/rmcs_msgs.hpp> // IWYU pragma: keep
#include <rmcs_msgs/sentry_event.hpp>
#include <std_msgs/msg/bool.hpp>

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

    details::LuaContext lua{*this};
    details::Navigation nav{*this};
    details::RmcsContext rmcs{*this};

    bool latest_auto_climb_success_ = false;
    bool latest_support_arm_success_ = false;

    MotionFsm motion{*this};

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;
        using SentryEvent = rmcs_msgs::SentryEvent;

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_toward;
        OutputInterface<std::unordered_map<SentryEvent, std::uint16_t>> sentry_events;

        explicit Command(Navigation& component) {
            component.register_output("/rmcs_navigation/enable_control", enable_control, true);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, true);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output("/rmcs_navigation/chassis_velocity", chassis_speed, kVecNaN);
            component.register_output("/rmcs_navigation/gimbal_toward", gimbal_toward, kVecNaN);
            component.register_output("/rmcs_navigation/sentry_events", sentry_events);
        }
    } command{*this};

    auto sync_blackboard() {
        auto [x, y, yaw] = nav.check_position();

        if (std::isnan(yaw)) {
            yaw = motion.context.current_local_yaw;
        }

        motion.context.current_world_yaw = yaw;
        motion.context.x = x;
        motion.context.y = y;

        auto& blackboard = lua.blackboard();

        auto user = blackboard["user"].get<sol::table>();
        user["health"] = *rmcs.robot_health;
        user["bullet"] = *rmcs.robot_bullet;
        user["chassis_power_limit"] = *rmcs.chassis_power_limit_referee;
        user["should_shoot"] = *rmcs.should_shoot;
        user["x"] = x;
        user["enemy_visible"] = rmcs.enemy_center.ready() && (*rmcs.enemy_center).allFinite()
                             && !(*rmcs.enemy_center).isZero();
        user["y"] = y;
        user["yaw"] = yaw;
        user["auto_climb_success"] = latest_auto_climb_success_;
        user["support_arm_success"] = latest_support_arm_success_;

        auto game = blackboard["game"].get<sol::table>();
        game["stage"] = rmcs_msgs::to_string(*rmcs.game_stage);

        auto play = blackboard["play"].get<sol::table>();
        play["rswitch"] = rmcs_msgs::to_string(*rmcs.switch_right);
        play["lswitch"] = rmcs_msgs::to_string(*rmcs.switch_left);

        auto autoaim = blackboard["autoaim"].get<sol::table>();
        autoaim["should_control"] = *rmcs.auto_aim_should_control;

        auto meta = blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), node::option()} {

        rmcs.init();

        climb_trigger_pub_ =
            create_publisher<std_msgs::msg::Bool>("/rmcs_navigation/climb_trigger", 10);
        support_arm_trigger_pub_ =
            create_publisher<std_msgs::msg::Bool>("/rmcs_navigation/support_arm_trigger", 10);

        auto_climb_success_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/chassis/climber/auto_climb_success", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                latest_auto_climb_success_ = msg->data;
            });
        support_arm_success_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/chassis/support_arm/success", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                latest_support_arm_success_ = msg->data;
            });

        lua.inject(
            "update_enable_control", [this](bool enable) { *command.enable_control = enable; });
        lua.inject("send_target", [this](double x, double y) { nav.send_target(x, y); });
        lua.inject(
            "switch_topic_forward", [this](bool enable) { nav.switch_topic_forward(enable); });
        lua.inject("update_gimbal_direction", [this](double yaw, double pitch) {
            motion.context.target_gimbal_toward = {yaw, pitch};
        });
        lua.inject(
            "switch_motion_mode", [this](const std::string& mode) { motion.switch_mode(mode); });
        lua.inject("update_under_attack", [this](bool yes) { motion.context.under_attack = yes; });
        lua.inject("push_sentry_event", [this](uint8_t event) {
            auto e = static_cast<rmcs_msgs::SentryEvent>(event);
            (*command.sentry_events)[e] += 1;
        });
        lua.inject("update_enable_autoaim", [this](bool enable) {
            *command.enable_autoaim = enable;
        });
        lua.inject("relocalize", [this] {
            const auto robot_id = *rmcs.robot_id;
            nav.relocalize(
                robot_id == rmcs_msgs::RobotId::UNKNOWN ? rmcs_msgs::RobotColor::UNKNOWN
                                                        : robot_id.color());
        });

        node::info("Navigation is initialized");
    }

    auto before_updating() -> void override { rmcs.ensure_defaults(); }

    auto update() -> void override {
        if (lua_tick_count++ == 10) [[unlikely]] {
            lua_tick_count = 0;
            sync_blackboard();
            lua.tick();
        }

        {
            *command.chassis_speed = Eigen::Vector2d::Zero();
            *command.gimbal_toward = Eigen::Vector2d::Zero();
            *command.chassis_behavior = motion.context.under_attack
                                          ? rmcs_msgs::ChassisMode::SPIN_FAST
                                          : rmcs_msgs::ChassisMode::SPIN_SLOW;
        }

        const auto nav_cmd = nav.current_command();
        const auto elapsed = std::chrono::steady_clock::now() - nav_cmd.timestamp;
        motion.context.target_chassis_speed =
            (elapsed > kCmdVelTimeout) ? Eigen::Vector2d::Zero() : nav_cmd.speed;

        const auto direction = fast_tf::cast<rmcs_description::OdomGimbalImu>(
            rmcs_description::BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()}, *rmcs.tf);
        motion.context.current_local_yaw = std::atan2(direction->y(), direction->x());

        const auto cmd = motion.spin_once();
        if (*command.enable_control) {
            *command.chassis_speed = cmd.chassis_speed;
            *command.gimbal_toward = cmd.gimbal_toward;
            *command.chassis_behavior = cmd.chassis_mode;

            auto mode = cmd.chassis_mode;

            if (mode == ChassisMode::CLIMB && !climb_trigger_active_) {
                climb_trigger_active_ = true;
                latest_auto_climb_success_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                climb_trigger_pub_->publish(msg);
            } else if (mode != ChassisMode::CLIMB && climb_trigger_active_) {
                climb_trigger_active_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                climb_trigger_pub_->publish(msg);
            }

            if (mode == ChassisMode::SUPPORT_ARM && !support_arm_trigger_active_) {
                support_arm_trigger_active_ = true;
                latest_support_arm_success_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                support_arm_trigger_pub_->publish(msg);
            } else if (mode != ChassisMode::SUPPORT_ARM && support_arm_trigger_active_) {
                support_arm_trigger_active_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                support_arm_trigger_pub_->publish(msg);
            }
        }
    }

private:
    using ChassisMode = rmcs_msgs::ChassisMode;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr climb_trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr support_arm_trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_climb_success_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr support_arm_success_sub_;

    bool climb_trigger_active_ = false;
    bool support_arm_trigger_active_ = false;
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
