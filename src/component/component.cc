#include "util/command.hh"
#include "util/status.hh"

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>

#include <yaml-cpp/yaml.h>

namespace rmcs_navigation {

constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node {
private:
    /// RCLCPP
    using Twist = geometry_msgs::msg::Twist;
    using String = std_msgs::msg::String;

    std::shared_ptr<rclcpp::Subscription<Twist>> subscription_twist;
    std::shared_ptr<rclcpp::Subscription<String>> subscription_command;

    std::shared_ptr<rclcpp::Publisher<String>> publisher_status;

    using Trigger = std_srvs::srv::Trigger;
    std::shared_ptr<rclcpp::Service<Trigger>> referee_status_service;

    template <typename... Args>
    auto info(std::format_string<Args...> fmt, Args&&... args) const {
        auto string = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_INFO(get_logger(), "%s", string.c_str());
    }
    template <typename... Args>
    auto warn(std::format_string<Args...> fmt, Args&&... args) const {
        auto string = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_WARN(get_logger(), "%s", string.c_str());
    }
    template <typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) const {
        auto string = std::format(fmt, std::forward<Args>(args)...);
        RCLCPP_ERROR(get_logger(), "%s", string.c_str());
    }

    /// RMCS
    std::chrono::steady_clock::time_point command_received_timestamp;
    std::chrono::milliseconds timeout_interval{100};
    std::atomic<bool> has_warning_timeout = false;

    Eigen::Vector2d scanning_angle_speed = {kNan, kNan};
    OutputInterface<Eigen::Vector2d> command_chassis_velocity;
    OutputInterface<Eigen::Vector2d> command_gimbal_velocity;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;

private:
    auto set_gimbal_mode(GimbalMode mode) {
        // TODO:
        // 即 cmd_vel_smoothed 的方向
        std::ignore = this;
        std::ignore = mode;
    }
    auto set_gimbal_scanning_area(double begin, double final) {
        // TODO:
        // 扫描模式下，云台不跟随前进方向，上下扫描，Yaw 按照给定角度扫描
        // Begin -> Final -> Begin -> ...
        std::ignore = this;
        std::ignore = begin;
        std::ignore = final;
    }

    auto referee_status_service_callback(
        const std::shared_ptr<Trigger::Request>&,
        const std::shared_ptr<Trigger::Response>& response) const {

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Referee Status");
        text("-  stage: {}", rmcs_navigation::to_string(*game_stage));
        text("- health: {}", *robot_health);
        text("- bullet: {}", *robot_bullet);

        response->success = true;
        response->message = feedback_message.str();
    }

    auto subscription_twist_callback(const std::unique_ptr<Twist>& msg) {
        command_chassis_velocity->x() = msg->linear.x;
        command_chassis_velocity->y() = msg->linear.y;

        command_gimbal_velocity->x() = msg->angular.z;
        command_gimbal_velocity->y() = 0;

        command_received_timestamp = std::chrono::steady_clock::now();
        has_warning_timeout = false;
    }

    bool has_command_error = false;
    auto subscription_command_callback(const std::unique_ptr<String>& msg) {
        auto error_once = [this](const std::string& string) {
            if (!has_command_error)
                error("Nav command error: {}", string);
            has_command_error = true;
        };

        try {
            auto yaml = YAML::Load(msg->data);
            auto command = Command{yaml};

            set_gimbal_mode(command.gimbal_mode);

        } catch (const std::exception& e) {
            error_once(e.what());
        }
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name()} {

        // RMCS
        const auto vec_nan = Eigen::Vector2d{kNan, kNan};
        Component::register_output(
            std::format("/{}/chassis_velocity", get_component_name()), command_chassis_velocity,
            vec_nan);
        Component::register_output(
            std::format("/{}/gimbal_velocity", get_component_name()), command_gimbal_velocity,
            vec_nan);

        Component::register_input("/referee/game/stage", game_stage, true);
        Component::register_input("/referee/current_hp", robot_health, true);
        Component::register_input("/referee/shooter/bullet_allowance", robot_bullet, true);

        // NAV2
        publisher_status =
            Node::create_publisher<String>(std::format("/{}/status", get_component_name()), 10);

        subscription_twist = Node::create_subscription<Twist>(
            "/cmd_vel_smoothed", 0,
            [this](const std::unique_ptr<Twist>& msg) { subscription_twist_callback(msg); });
        subscription_command = Node::create_subscription<String>(
            std::format("/{}/command", get_component_name()), 0,
            [this](const std::unique_ptr<String>& msg) { subscription_command_callback(msg); });

        referee_status_service = Node::create_service<Trigger>(
            std::format("/{}/service/referee_status", get_component_name()),
            [this](
                const std::shared_ptr<Trigger::Request>& request,
                const std::shared_ptr<Trigger::Response>& response) {
                referee_status_service_callback(request, response);
            });
        command_received_timestamp = std::chrono::steady_clock::now();
    }

    auto update() -> void override {

        using namespace std::chrono_literals;
        const auto interval = std::chrono::steady_clock::now() - command_received_timestamp;
        if (interval > timeout_interval) {
            if (has_warning_timeout == false) {
                has_warning_timeout = true;
                warn("Lost navigation control, reset command velocity now");
            }
            *command_chassis_velocity = Eigen::Vector2d::Zero();
            *command_gimbal_velocity = Eigen::Vector2d::Zero();
        }
    }
};

} // namespace rmcs_navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_navigation::Navigation, rmcs_executor::Component)
