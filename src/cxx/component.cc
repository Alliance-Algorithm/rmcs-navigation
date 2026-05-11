#include "cxx/context.hh"
#include "cxx/controller/normal.hh"
#include "cxx/lua_context.hh"
#include "cxx/navigation.hh"
#include "cxx/util/localization/engine.hh"
#include "cxx/util/node_mixin.hh"

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
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
    details::Context context{*this, *this};
    std::unique_ptr<Localization> localization;
    bool relocalization_enabled = true;

    // 控制器框架
    IController* selected_controller = nullptr;
    std::unordered_map<std::string, std::unique_ptr<IController>> controllers;

    Eigen::Vector2d desired_direction = Eigen::Vector2d::Zero();
    double current_world_yaw = std::numeric_limits<double>::quiet_NaN();

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_speed;

        auto init(Navigation& component) -> void {
            component.register_output("/rmcs_navigation/enable_control", enable_control, false);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, false);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_velocity", gimbal_speed, Eigen::Vector2d::Zero());
        }
    } command;

private:
    static auto option() noexcept {
        return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    }

    auto sync_blackboard() {
        const auto [x, y, yaw] = navigation.check_position();
        current_world_yaw = yaw; // 高频查询 TF 是不对的，所以应该先缓存一份

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
        relocalization_enabled = has_parameter("enable_relocalization")
                                   ? get_parameter_or<bool>("enable_relocalization", true)
                                   : true;
        if (relocalization_enabled) {
            auto config = Localization::Config{.rclcpp = *this};
            if (has_parameter("localization.service_name"))
                config.service_name = get_parameter("localization.service_name").as_string();
            if (has_parameter("localization.request_timeout_sec"))
                config.request_timeout_sec =
                    get_parameter("localization.request_timeout_sec").as_double();
            localization = std::make_unique<Localization>(std::move(config));
        } else {
            logging::warn("relocalization is disabled by parameter enable_relocalization=false");
        }

        context.init(io_mutex, mock_context);
        command.init(*this);

        lua_context.init({
            .update_enable_control = [this](bool enable) { *command.enable_control = enable; },
            .send_target = [this](double x, double y) { navigation.send_target(x, y); },
            .switch_topic_forward =
                [this](bool enable) { navigation.switch_topic_forward(enable); },
            .update_gimbal_direction = [this](double angle) { desired_direction = {angle, 0.0}; },
            .switch_controller =
                [this](const std::string& mode) {
                    if (!controllers.contains(mode)) {
                        selected_controller = nullptr;
                        logging::fuck("controller '{}' not found", mode);
                        return;
                    }
                    selected_controller = controllers.at(mode).get();
                    logging::info("switched to controller '{}'", mode);
                },
            .relocalize_initial =
                [this](double x, double y, double yaw) {
                    if (!relocalization_enabled || !localization) {
                        logging::warn("relocalize_initial ignored: disabled");
                        return false;
                    }
                    return localization->relocalize(RelocalizeMode::Initial, x, y, yaw);
                },
            .relocalize_local =
                [this](double x, double y, double yaw) {
                    if (!relocalization_enabled || !localization) {
                        logging::warn("relocalize_local ignored: disabled");
                        return false;
                    }
                    return localization->relocalize(RelocalizeMode::Local, x, y, yaw);
                },
            .relocalize_wide =
                [this](double x, double y, double yaw) {
                    if (!relocalization_enabled || !localization) {
                        logging::warn("relocalize_wide ignored: disabled");
                        return false;
                    }
                    return localization->relocalize(RelocalizeMode::Wide, x, y, yaw);
                },
            .relocalize_status =
                [this] {
                    if (!relocalization_enabled || !localization) {
                        return RelocalizeStatus{
                            .state = RelocalizeState::FAILED,
                            .success = false,
                            .message = "disabled",
                        };
                    }
                    return localization->relocalize_status();
                },
        });

        controllers["normal"] = std::make_unique<NormalController>();
        selected_controller = controllers["normal"].get();

        logging::info("Navigation is initialized");
    }

    auto before_updating() -> void override {
        if (auto ok = context.health(); !ok) {
            logging::fuck("{}", ok.error());
            throw std::runtime_error{"Context Error"};
        }
    }

    auto update() -> void override {
        if (lua_tick_count++ == 100) [[unlikely]] {
            lua_tick_count = 0;
            auto lock = std::scoped_lock{io_mutex};
            sync_blackboard();
            lua_context.tick();
        }

        if (selected_controller && *command.enable_control) {
            const auto nav_cmd = navigation.current_command();
            const auto elapsed = std::chrono::steady_clock::now() - nav_cmd.timestamp;
            const auto effective_vel =
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
            const auto current_local_yaw = std::atan2(vector.y(), vector.x());

            selected_controller->update_context({
                .target_chassis_speed = effective_vel,
                .target_gimbal_toward = desired_direction,
                .current_local_yaw = current_local_yaw,
                .current_world_yaw = current_world_yaw,
            });

            auto cmd = selected_controller->generate_command();
            *command.chassis_speed = cmd.chassis_speed;
            *command.gimbal_speed = cmd.gimbal_toward;
            *command.chassis_behavior = cmd.chassis_mode;
        } else {
            *command.chassis_speed = Eigen::Vector2d::Zero();
            *command.gimbal_speed = Eigen::Vector2d::Zero();
        }
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
