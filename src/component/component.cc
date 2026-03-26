#include "component/decision/plan.hh"
#include "component/util/logger_mixin.hh"
#include "component/util/navigation_restarter.hh"
#include "component/util/nod_task_queue.hh"
#include "component/util/rmcs_msgs_format.hh" // IWYU pragma: keep
#include "component/util/switch_event_detector.hh"

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs::navigation {

constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::LoggerMixin {
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

    // tx, ty, rx 用于导航，ry 用于点头事件
    OutputInterface<Eigen::Vector2d> command_chassis_velocity;
    OutputInterface<Eigen::Vector2d> command_gimbal_velocity;

    OutputInterface<bool> command_rotate_chassis;
    OutputInterface<bool> command_gimbal_scanning;
    OutputInterface<bool> command_enable_autoaim;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;
    InputInterface<uint32_t> red_score;
    InputInterface<uint32_t> blue_score;
    InputInterface<rmcs_msgs::Switch> switch_right;
    InputInterface<rmcs_msgs::Switch> switch_left;

    std::string navigation_config_name = "rmul";

    NavigationRestarter navigation_restarter{
        [this](const std::string& msg) { this->info("{}", msg); }};
    NodTaskQueue nod_task_queue{[this](double pitch_velocity) {
        if (command_gimbal_velocity.active()) {
            command_gimbal_velocity->y() = pitch_velocity;
        }
    }};
    SwitchEventDetector right_switch_detector{switch_right};

    std::chrono::steady_clock::time_point last_twist_timestamp;
    bool has_last_twist_timestamp = false;

    /// DECISION
    PlanBox plan_box;

    Eigen::Vector2d last_goal_position = Eigen::Vector2d::Zero();
    rmcs_msgs::GameStage last_game_stage = rmcs_msgs::GameStage::UNKNOWN;

    bool enable_fallback_mode = false;

private:
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
        auto compensated_x = msg->linear.x;
        auto compensated_y = msg->linear.y;

        auto timestamp_now = std::chrono::steady_clock::now();
        if (has_last_twist_timestamp) {
            auto dt = std::chrono::duration<double>(timestamp_now - last_twist_timestamp).count();
            dt = std::clamp(dt, 0.0, 0.2);

            auto delta_yaw = std::clamp(msg->angular.z * dt, -1.0, 1.0);

            auto velocity = Eigen::Vector2d{msg->linear.x, msg->linear.y};
            auto compensated_velocity = Eigen::Rotation2Dd{+delta_yaw} * velocity;
            compensated_x = compensated_velocity.x();
            compensated_y = compensated_velocity.y();
        }
        last_twist_timestamp = timestamp_now;
        has_last_twist_timestamp = true;

        command_chassis_velocity->x() = compensated_x;
        command_chassis_velocity->y() = compensated_y;

        command_gimbal_velocity->x() = msg->angular.z;

        command_received_timestamp = std::chrono::steady_clock::now();
        has_warning_timeout = false;
    }

    auto spin_plan_box() {
        // 此为安全模式，不进行导航，原地旋转加扫描
        if (enable_fallback_mode) {
            *command_gimbal_scanning = true;
            *command_rotate_chassis = true;
            *command_enable_autoaim = true;
            return;
        }

        auto position = check_current_position();

        using Information = PlanBox::Information;
        plan_box.update_information([position, this](Information& information) {
            information.game_stage = *game_stage;

            auto [x, y] = position;
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
            // 目标点相同且间隔在一定秒数内，跳过
            constexpr auto k_tolerance = 1e-2;
            constexpr auto k_interval = std::chrono::seconds{5};
            if (std::abs(last_goal_position.x() - goal_x) < k_tolerance
                && std::abs(last_goal_position.y() - goal_y) < k_tolerance) {
                if (std::chrono::steady_clock::now() - last_navigate_timestamp < k_interval)
                    break;
            }
            set_goal_position(goal_x, goal_y);

            last_navigate_timestamp = std::chrono::steady_clock::now();
            last_goal_position = Eigen::Vector2d{goal_x, goal_y};
        } while (false);

        *command_gimbal_scanning = plan_box.gimbal_scanning();
        *command_rotate_chassis = plan_box.rotate_chassis();
    }

    auto enqueue_nod_sequence() -> void {
        using namespace std::chrono_literals;

        nod_task_queue.push_delay(300ms);
        for (auto i = 0; i < 2; ++i) {
            nod_task_queue.nod_once(0.8, 220ms);
        }
        nod_task_queue.push_delay(1s);
        nod_task_queue.push_task(1ms, {}, [this] {
            info("Nod sequence finished, restart navigation now");
            navigation_restarter.start_async(navigation_config_name);
        });
    }

public:
    explicit Navigation()
        : rclcpp::Node{
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)} {

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        // RMCS
        const auto kNanVec = Eigen::Vector2d{kNan, kNan};
        Component::register_output(
            "/rmcs_navigation/chassis_velocity", command_chassis_velocity, kNanVec);
        Component::register_output(
            "/rmcs_navigation/gimbal_velocity", command_gimbal_velocity, kNanVec);
        Component::register_output(
            "/rmcs_navigation/rotate_chassis", command_rotate_chassis, false);
        Component::register_output(
            "/rmcs_navigation/gimbal_scanning", command_gimbal_scanning, false);
        Component::register_output(//
            "/rmcs_navigation/start_autoaim", command_enable_autoaim, false);

        Component::register_input("/referee/game/stage", game_stage, true);
        Component::register_input("/referee/id", robot_id, true);
        Component::register_input("/referee/current_hp", robot_health, true);
        Component::register_input("/referee/shooter/bullet_allowance", robot_bullet, true);
        Component::register_input("/referee/game/red_score", red_score, true);
        Component::register_input("/referee/game/blue_score", blue_score, true);
        Component::register_input("/remote/switch/right", switch_right, true);
        Component::register_input("/remote/switch/left", switch_left, true);

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

        navigation_config_name = get_parameter_or<std::string>("config_name", "rmul");
        if (navigation_config_name.empty()) {
            error("Parameter 'config_name' is empty, fallback to 'rmul'");
            rclcpp::shutdown();
        }

        auto path = ament_index_cpp::get_package_share_directory("rmcs-navigation");
        auto config_file =
            std::filesystem::path{path} / "config" / std::format("{}.yaml", navigation_config_name);
        try {
            auto config = YAML::LoadFile(config_file.string());
            auto result = plan_box.configure(config["decision"]);
            if (!result) {
                error("Configure error: {}", result.error());
                rclcpp::shutdown();
            }
            info("Loaded decision config: {}", config_file.string());
        } catch (const std::exception& exception) {
            error(
                "Failed to load decision config '{}' : {}", config_file.string(), exception.what());
            rclcpp::shutdown();
        }

        using namespace std::chrono_literals;
        plan_scheduler = Node::create_wall_timer(100ms, [this] { spin_plan_box(); });
    }

    auto update() -> void override {
        using namespace std::chrono_literals;
        const auto now = std::chrono::steady_clock::now();
        const auto interval = now - command_received_timestamp;
        if (interval > timeout_interval) {
            if (has_warning_timeout == false) {
                has_warning_timeout = true;
                warn("Lost navigation control, reset command velocity now");
            }
            *command_chassis_velocity = Eigen::Vector2d::Zero();
            *command_gimbal_velocity = Eigen::Vector2d::Zero();
        }

        using rmcs_msgs::GameStage;
        using rmcs_msgs::Switch;
        if ((last_game_stage != GameStage::STARTED) //
            && (*game_stage == GameStage::STARTED)) {
            if (*switch_right != Switch::UP) {
                enable_fallback_mode = true;
                warn("Fallback mode detected, runing without navigation");
            }
        }

        if (right_switch_detector.spin(now)) {
            info("Right switch trigger detected, enqueue nod sequence");
            enqueue_nod_sequence();
        }

        nod_task_queue.spin(now);

        // .....
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
