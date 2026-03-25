#include "component/decision/plan.hh"
#include "component/util/rmcs_msgs_format.hh" // IWYU pragma: keep

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include <yaml-cpp/yaml.h>

namespace rmcs_navigation {

constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
static auto kRclcppOption =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

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

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;
    std::shared_ptr<NavigateToPoseClient> client;
    std::chrono::steady_clock::time_point last_navigate_timestamp;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<rclcpp::TimerBase> plan_scheduler;

    /// RMCS
    std::chrono::steady_clock::time_point command_received_timestamp;
    std::chrono::milliseconds timeout_interval{100};
    std::atomic<bool> has_warning_timeout = false;

    Eigen::Vector2d scanning_angle_speed = {kNan, kNan};
    OutputInterface<Eigen::Vector2d> command_chassis_velocity;
    OutputInterface<Eigen::Vector2d> command_gimbal_velocity;
    OutputInterface<bool> rotate_chassis;
    OutputInterface<bool> gimbal_scanning;
    OutputInterface<bool> start_autoaim;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;
    InputInterface<uint32_t> red_score;
    InputInterface<uint32_t> blue_score;

    /// DECISION
    PlanBox plan_box;

    Eigen::Vector2d last_goal_position = Eigen::Vector2d::Zero();

private:
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

    auto set_gimbal_scanning_area(double begin, double final) {
        // TODO:
        // 扫描模式下，云台不跟随前进方向，上下扫描，Yaw 按照给定角度扫描
        // Begin -> Final -> Begin -> ...
        std::ignore = this;
        std::ignore = begin;
        std::ignore = final;
    }

    auto check_current_position() const noexcept -> std::tuple<double, double> {
        try {
            const auto transform =
                tf_buffer->lookupTransform("world", "base_link", rclcpp::Time{0});
            return std::tuple{
                transform.transform.translation.x,
                transform.transform.translation.y,
            };
        } catch (const std::exception&) {
            return std::tuple{kNan, kNan};
        }
    }

    auto set_goal_position(double x, double y) {
        client->async_cancel_all_goals();

        auto goal = NavigateToPose::Goal{};
        goal.pose.header.stamp = now();
        goal.pose.header.frame_id = "world";
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        client->async_send_goal(goal);
        info("Goal position updated: ({}, {})", x, y);
    }

    auto referee_status_service_callback(
        const std::shared_ptr<Trigger::Request>&,
        const std::shared_ptr<Trigger::Response>& response) const {

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Referee Status");
        text("-     id: {}", *robot_id);
        text("-  stage: {}", rmcs_msgs::to_string(*game_stage));
        text("- health: {}", *robot_health);
        text("- bullet: {}", *robot_bullet);
        // text("- bscore: {}", *blue_score);
        // text("- rscore: {}", *red_score);

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

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), kRclcppOption} {

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        // RMCS
        const auto vec_nan = Eigen::Vector2d{kNan, kNan};
        Component::register_output(
            "/rmcs_navigation/chassis_velocity", command_chassis_velocity, vec_nan);
        Component::register_output(
            "/rmcs_navigation/gimbal_velocity", command_gimbal_velocity, vec_nan);
        Component::register_output("/rmcs_navigation/rotate_chassis", rotate_chassis, false);
        Component::register_output("/rmcs_navigation/gimbal_scanning", gimbal_scanning, false);
        Component::register_output("/rmcs_navigation/start_autoaim", start_autoaim, false);

        Component::register_input("/referee/game/stage", game_stage, true);
        Component::register_input("/referee/id", robot_id, true);
        Component::register_input("/referee/current_hp", robot_health, true);
        Component::register_input("/referee/shooter/bullet_allowance", robot_bullet, true);
        Component::register_input("/referee/game/red_score", red_score, true);
        Component::register_input("/referee/game/blue_score", blue_score, true);

        // NAV2
        subscription_twist = Node::create_subscription<Twist>(
            "/cmd_vel_smoothed", 0,
            [this](const std::unique_ptr<Twist>& msg) { subscription_twist_callback(msg); });

        referee_status_service = Node::create_service<Trigger>(
            std::format("/{}/service/referee_status", get_component_name()),
            [this](
                const std::shared_ptr<Trigger::Request>& request,
                const std::shared_ptr<Trigger::Response>& response) {
                referee_status_service_callback(request, response);
            });
        command_received_timestamp = std::chrono::steady_clock::now();

        // DECISION
        // 从 config 中获取配置
        plan_box.set_printer([this](const std::string& msg) { info("PlanBox: {}", msg); });

        auto name = get_parameter("config_name").as_string();
        if (name.empty()) {
            error("Parameter 'config_name' is empty, fallback to '{}'", name);
            rclcpp::shutdown();
        }
        plan_box.set_config_name(name);

        auto path = ament_index_cpp::get_package_share_directory("rmcs-navigation");
        auto config_file = std::filesystem::path{path} / "config" / std::format("{}.yaml", name);
        try {
            auto config = YAML::LoadFile(config_file.string());
            if (config["decision"]) {
                plan_box.configure(config["decision"]);
            } else {
                plan_box.configure(config);
            }
            info("Loaded decision config: {}", config_file.string());
        } catch (const std::exception& exception) {
            error(
                "Failed to load decision config '{}' : {}", config_file.string(), exception.what());
        }

        using namespace std::chrono_literals;
        plan_scheduler = Node::create_wall_timer(100ms, [this] {
            auto [x, y] = check_current_position();

            plan_box.update_information([=, this](PlanBox::Information& information) {
                information.game_stage = *game_stage;

                information.current_x = x;
                information.current_y = y;

                information.health = *robot_health;
                information.bullet = *robot_bullet;
            });

            do {
                auto [goal_x, goal_y] = plan_box.goal_position();
                // 非法目标点，跳过
                if (std::isnan(goal_x) || std::isnan(goal_y)) {
                    break;
                }
                // 目标点相同且间隔在 2s 以下，跳过
                if (std::abs(last_goal_position.x() - x) < 1e-2
                    && std::abs(last_goal_position.y() - y) < 1e-2) {
                    auto interval = std::chrono::seconds{2};
                    auto diff = std::chrono::steady_clock::now() - last_navigate_timestamp;
                    if (diff < interval)
                        break;
                }
                set_goal_position(goal_x, goal_y);

                last_navigate_timestamp = std::chrono::steady_clock::now();
                last_goal_position = Eigen::Vector2d{x, y};
            } while (false);

            *gimbal_scanning = plan_box.gimbal_scanning();
            *rotate_chassis = plan_box.rotate_chassis();
        });
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
