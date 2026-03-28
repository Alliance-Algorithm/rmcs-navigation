#include "component/decision/plan.hh"
#include "component/util/logger_mixin.hh"
#include "component/util/navigation_screen.hh"
#include "component/util/nod_task_queue.hh"
#include "component/util/rmcs_msgs_format.hh" // IWYU pragma: keep
#include "component/util/switch_event_detector.hh"
#include "component/util/tie.hh"
#include "component/util/value_enter_detector.hh"

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs::navigation {

constexpr auto kNanLocal = std::numeric_limits<double>::quiet_NaN();

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::LoggerMixin {
private:
    mutable std::mutex io_mutex;
    bool use_mock_interface = false;

    /// RCLCPP
    using Twist = geometry_msgs::msg::Twist;
    using String = std_msgs::msg::String;

    std::shared_ptr<rclcpp::Subscription<Twist>> subscription_twist;
    std::shared_ptr<rclcpp::Subscription<String>> subscription_command;

    std::shared_ptr<rclcpp::Publisher<String>> publisher_status;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> publisher_goal;

    using Trigger = std_srvs::srv::Trigger;
    std::shared_ptr<rclcpp::Service<Trigger>> referee_status_service;

    std::chrono::steady_clock::time_point last_navigate_timestamp;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<rclcpp::TimerBase> plan_scheduler;

    /// RMCS
    std::chrono::steady_clock::time_point command_received_timestamp;
    std::chrono::milliseconds timeout_interval{100};
    std::atomic<bool> has_warning_timeout = false;

    Eigen::Vector2d scanning_angle_speed = {kNanLocal, kNanLocal};

    // tx, ty, rx 用于导航，ry 用于点头事件
    OutputInterface<Eigen::Vector2d> command_chassis_velocity;
    OutputInterface<Eigen::Vector2d> command_gimbal_velocity;

    OutputInterface<bool> command_rotate_chassis;
    OutputInterface<bool> command_detect_targets;
    OutputInterface<bool> command_enable_autoaim;

    struct Context {
        InputInterface<rmcs_msgs::GameStage> game_stage;
        InputInterface<rmcs_msgs::RobotId> robot_id;
        InputInterface<std::uint16_t> robot_health;
        InputInterface<std::uint16_t> robot_bullet;
        InputInterface<std::uint32_t> red_score;
        InputInterface<std::uint32_t> blue_score;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;

        auto init(rmcs_executor::Component& component, bool mock = false) -> void {
            if (!mock) {
                component.register_input("/referee/game/stage", game_stage, true);
                component.register_input("/referee/current_hp", robot_health, true);
                component.register_input("/referee/shooter/bullet_allowance", robot_bullet, true);
                component.register_input("/referee/game/red_score", red_score, true);
                component.register_input("/referee/game/blue_score", blue_score, true);
            } else {
                game_stage.make_and_bind_directly();
                robot_health.make_and_bind_directly();
                robot_bullet.make_and_bind_directly();
                red_score.make_and_bind_directly();
                blue_score.make_and_bind_directly();
            }

            component.register_input("/referee/id", robot_id, true);
            component.register_input("/remote/switch/right", switch_right, true);
            component.register_input("/remote/switch/left", switch_left, true);
        }
        auto update(const std::string& raw_data) -> void {
            // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
            auto data = YAML::Load(raw_data);
            if (auto node = data["game_stage"]; node && game_stage.ready())
                const_cast<rmcs_msgs::GameStage&>(*game_stage) =
                    static_cast<rmcs_msgs::GameStage>(node.as<int>());
            if (auto node = data["robot_health"]; node && robot_health.ready())
                const_cast<std::uint16_t&>(*robot_health) = node.as<std::uint16_t>();
            if (auto node = data["robot_bullet"]; node && robot_bullet.ready())
                const_cast<std::uint16_t&>(*robot_bullet) = node.as<std::uint16_t>();
            if (auto node = data["red_score"]; node && red_score.ready())
                const_cast<std::uint32_t&>(*red_score) = node.as<std::uint32_t>();
            if (auto node = data["blue_score"]; node && blue_score.ready())
                const_cast<std::uint32_t&>(*blue_score) = node.as<std::uint32_t>();
            // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
        }
    } context;

    std::string navigation_config_name = "rmul";

    NavigationScreen screen{[this](const std::string& msg) { this->info("{}", msg); }};

    NodTaskQueue nod_task_queue{[this](double pitch_velocity) {
        if (command_gimbal_velocity.active()) {
            command_gimbal_velocity->y() = pitch_velocity;
        }
    }};
    SwitchEventDetector right_switch_detector{context.switch_right};

    std::chrono::steady_clock::time_point last_twist_timestamp;
    bool has_last_twist_timestamp = false;

    /// DECISION
    PlanBox plan_box;

    Eigen::Vector2d last_goal_position = Eigen::Vector2d::Zero();
    ValueEnterDetector<rmcs_msgs::GameStage> started_detector{rmcs_msgs::GameStage::STARTED};

    bool enable_fallback_mode = false;

private:
    auto check_current_position() const noexcept -> std::tuple<double, double> {
        try {
            const auto transform =
                tf_buffer->lookupTransform("world", "base_link", rclcpp::Time{0});
            return std::tuple{
                transform.transform.translation.x,
                transform.transform.translation.y,
            };
        } catch (const std::exception&) {
            return std::tuple{kNanLocal, kNanLocal};
        }
    }

    auto update_goal_position(double x, double y) {
        auto goal = geometry_msgs::msg::PoseStamped{};
        goal.header.stamp = now();
        goal.header.frame_id = "world";

        util::tie(goal.pose.position) = std::tuple{x, y, 0.0};
        util::tie(goal.pose.orientation) = std::tuple{0.0, 0.0, 0.0, 1.0};

        publisher_goal->publish(goal);
        info("Goal position updated: ({}, {})", x, y);
    }

    auto referee_status_service_callback(
        const std::shared_ptr<Trigger::Request>&,
        const std::shared_ptr<Trigger::Response>& response) const {
        auto lock = std::scoped_lock{io_mutex};

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("Referee Status");
        text("-     id: {}", *context.robot_id);
        text("-  stage: {}", rmcs_msgs::to_string(*context.game_stage));
        text("- health: {}", *context.robot_health);
        text("- bullet: {}", *context.robot_bullet);
        // text("- bscore: {}", *blue_score);
        // text("- rscore: {}", *red_score);

        response->success = true;
        response->message = feedback_message.str();
    }

    auto subscription_twist_callback(const std::unique_ptr<Twist>& msg) {
        auto lock = std::scoped_lock{io_mutex};

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
        auto lock = std::scoped_lock{io_mutex};

        // 此为安全模式，不进行导航，原地旋转加扫描
        if (enable_fallback_mode) {
            *command_detect_targets = true;
            *command_rotate_chassis = true;
            *command_enable_autoaim = true;
            return;
        }

        auto position = check_current_position();

        using Information = PlanBox::Information;
        plan_box.update_information([position, this](Information& info) {
            info.game_stage = *context.game_stage;

            auto [x, y] = position;
            info.current_x = x;
            info.current_y = y;

            info.enemy_x = kNanLocal;
            info.enemy_y = kNanLocal;

            info.health = *context.robot_health;
            info.bullet = *context.robot_bullet;
        });
        plan_box.fetch_command([this](const PlanBox::Command& command) {
            auto [goal_x, goal_y] = std::tuple{command.goal_x, command.goal_y};
            // 非法目标点，跳过
            if (!std::isnan(goal_x) && !std::isnan(goal_y)) {
                // 目标点相同且间隔在一定秒数内，跳过
                constexpr auto kTolerance = 1e-2;
                constexpr auto kInterval = std::chrono::seconds{5};
                auto duplicated_goal = std::abs(last_goal_position.x() - goal_x) < kTolerance
                                    && std::abs(last_goal_position.y() - goal_y) < kTolerance;
                auto still_in_interval =
                    std::chrono::steady_clock::now() - last_navigate_timestamp < kInterval;
                if (!duplicated_goal || !still_in_interval) {
                    update_goal_position(goal_x, goal_y);
                    last_navigate_timestamp = std::chrono::steady_clock::now();
                    last_goal_position = Eigen::Vector2d{goal_x, goal_y};
                }
            }

            *command_detect_targets = command.detect_targets;
            *command_rotate_chassis = command.rotate_chassis;
            *command_enable_autoaim = command.enable_autoaim;
        });
    }

    auto enqueue_nod_sequence() -> void {
        using namespace std::chrono_literals;

        if (!nod_task_queue.empty())
            return;

        nod_task_queue.push_delay(300ms);
        for (auto i = 0; i < 2; ++i) {
            nod_task_queue.nod_once(0.8, 220ms);
        }
        nod_task_queue.push_delay(1s);
        nod_task_queue.push_task(1ms, {}, [this] {
            info("Nod sequence finished, restart navigation now");
            screen.restart_async();
        });
    }

public:
    explicit Navigation()
        : rclcpp::Node{
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)} {

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        publisher_goal =
            Node::create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);

        // RMCS
        const auto kNanVec = Eigen::Vector2d{kNanLocal, kNanLocal};
        register_output("/rmcs_navigation/chassis_velocity", command_chassis_velocity, kNanVec);
        register_output("/rmcs_navigation/gimbal_velocity", command_gimbal_velocity, kNanVec);
        register_output("/rmcs_navigation/rotate_chassis", command_rotate_chassis, false);
        register_output("/rmcs_navigation/detect_targets", command_detect_targets, false);
        register_output("/rmcs_navigation/start_autoaim", command_enable_autoaim, false);

        use_mock_interface = get_parameter_or("use_mock_interface", false);
        context.init(*this, use_mock_interface);

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

        if (use_mock_interface) {
            auto mock_context_topic = get_parameter_or<std::string>(
                "mock_context_topic", "/rmcs_navigation/mock_context");
            subscription_command = Node::create_subscription<String>(
                mock_context_topic, 10, [this](const std::unique_ptr<String>& msg) {
                    auto lock = std::scoped_lock{io_mutex};
                    try {
                        context.update(msg->data);
                    } catch (const std::exception& exception) {
                        warn("Failed to update mock context: {}", exception.what());
                    }
                });
            info("Mock context subscriber enabled on '{}'", mock_context_topic);
        }

        command_received_timestamp = std::chrono::steady_clock::now();

        // DECISION
        // 从 config 中获取配置
        plan_box.set_logging([this](const std::string& msg) { info("PlanBox: {}", msg); });

        navigation_config_name = get_parameter_or<std::string>("config_name", "rmul");
        if (navigation_config_name.empty()) {
            error("Parameter 'config_name' is empty, fallback to 'rmul'");
            rclcpp::shutdown();
        }
        screen.set_config_name(navigation_config_name);

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
        auto lock = std::scoped_lock{io_mutex};

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

        if (started_detector.spin(*context.game_stage)) {
            if (*context.switch_right == rmcs_msgs::Switch::MIDDLE) {
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
