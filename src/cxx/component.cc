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

namespace rmcs::navigation {

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::NodeMixin {
private:
    static constexpr auto kCmdVelTimeout = std::chrono::milliseconds{500};

    mutable std::mutex io_mutex;

    bool mock_context = false;

    std::atomic<std::uint16_t> lua_tick_count = 0;

    details::LuaContext lua_context{*this};
    details::Navigation navigation{*this};
    details::Context context{*this};

    MotionFsm motion{*this};

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_speed;

        auto init(Navigation& component) -> void {
            component.register_output("/rmcs_navigation/enable_control", enable_control, true);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, true);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_velocity", gimbal_speed, Eigen::Vector2d::Zero());
        }
    } command;

    static auto option() noexcept {
        return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    }

    auto sync_blackboard() {
        const auto [x, y, yaw] = navigation.check_position();

        // 高频查询 TF 是不对的，所以应该先缓存一份
        motion.context.current_world_yaw = yaw;
        motion.context.x = x;
        motion.context.y = y;

        auto& blackboard = lua_context.blackboard();

        auto user = blackboard["user"].get<sol::table>();
        user["health"] = *context.robot_health;
        user["bullet"] = *context.robot_bullet;
        user["chassis_power_limit"] = *context.chassis_power_limit_referee;
        user["x"] = x;
        user["y"] = y;
        user["yaw"] = yaw;

        auto game = blackboard["game"].get<sol::table>();
        game["stage"] = rmcs_msgs::to_string(*context.game_stage);

        auto play = blackboard["play"].get<sol::table>();
        play["rswitch"] = rmcs_msgs::to_string(*context.switch_right);
        play["lswitch"] = rmcs_msgs::to_string(*context.switch_left);

        auto meta = blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), option()} {

        mock_context = param<bool>("mock_context");

        context.init(io_mutex, mock_context);
        command.init(*this);

        lua_context.init({
            .update_enable_control = [this](bool enable) { *command.enable_control = enable; },
            .send_target = [this](double x, double y) { navigation.send_target(x, y); },
            .switch_topic_forward =
                [this](bool enable) { navigation.switch_topic_forward(enable); },
            .update_gimbal_direction =
                [this](double yaw, double pitch) {
                    motion.context.target_gimbal_toward = {yaw, pitch};
                },
            .switch_motion_mode = [this](const std::string& mode) { motion.switch_mode(mode); },
            .update_under_attack = [this](bool yes) { motion.context.under_attack = yes; },
        });
        motion.switch_mode("normal");

        logging::info("Navigation is initialized");
    }

    auto before_updating() -> void override {
        if (auto ok = context.health(); !ok) {
            logging::fuck("{}", ok.error());
            throw std::runtime_error{"Context Error"};
        }
    }

    auto update() -> void override {
        if (lua_tick_count++ == 10) [[unlikely]] {
            lua_tick_count = 0;
            auto lock = std::scoped_lock{io_mutex};
            sync_blackboard();
            lua_context.tick();
        }

        const auto use_command = [this](const MotionFsm::Command& cmd) {
            *command.chassis_speed = cmd.chassis_speed;
            *command.gimbal_speed = cmd.gimbal_toward;
            *command.chassis_behavior = cmd.chassis_mode;
        };
        const auto use_fallback = [this] {
            *command.chassis_speed = Eigen::Vector2d::Zero();
            *command.gimbal_speed = Eigen::Vector2d::Zero();
            *command.chassis_behavior = rmcs_msgs::ChassisMode::SPIN_FAST;
        };
        if (*command.enable_control) {
            const auto nav_cmd = navigation.current_command();
            const auto elapsed = std::chrono::steady_clock::now() - nav_cmd.timestamp;
            motion.context.target_chassis_speed =
                (elapsed > kCmdVelTimeout) ? Eigen::Vector2d::Zero() : nav_cmd.speed;

            const auto direction = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()},
                *context.tf);
            auto vector = *direction;
            vector.z() = 0.0;
            if (vector.norm() > 1e-9)
                vector.normalize();
            else
                vector = Eigen::Vector3d::UnitX();
            motion.context.current_local_yaw = std::atan2(vector.y(), vector.x());

            use_command(motion.spin_once());
        } else {
            use_fallback();
        }
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
