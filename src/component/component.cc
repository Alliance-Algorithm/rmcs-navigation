#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>

#include <yaml-cpp/yaml.h>

namespace rmcs {

constexpr auto kRosLabel = std::string_view{"rmcs_navigation"};
constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node {
private:
    /// RCLCPP
    using Twist = geometry_msgs::msg::Twist;
    std::shared_ptr<rclcpp::Subscription<Twist>> subscription_twist;

    using String = std_msgs::msg::String;
    std::shared_ptr<rclcpp::Publisher<String>> publisher_status;

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
    OutputInterface<Eigen::Vector3d> command_velocity;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;

    std::chrono::steady_clock::time_point command_received_timestamp;
    std::chrono::milliseconds timeout_interval{100};
    std::atomic<bool> has_warning_timeout = false;

public:
    explicit Navigation()
        : Node{kRosLabel.data()} {

        // RMCS
        const auto name = std::format("/{}/command_velocity", kRosLabel);
        const auto vec_nan = Eigen::Vector3d{kNan, kNan, kNan};
        Component::register_output(name, command_velocity, vec_nan);

        Component::register_input("/referee/game/stage", game_stage, true);
        // TODO:
        Component::register_input("/referee/health", robot_health, true);
        Component::register_input("/referee/shooter/bullet_allowance", robot_bullet, true);

        // NAV2
        subscription_twist = Node::create_subscription<Twist>(
            "/cmd_vel_smoothed", 10, [&, this](const std::unique_ptr<Twist>& msg) {
                command_velocity->x() = msg->linear.x;
                command_velocity->y() = msg->linear.y;
                command_velocity->z() = msg->angular.z;
                command_received_timestamp = std::chrono::steady_clock::now();
                has_warning_timeout = false;
            });
        publisher_status = Node::create_publisher<String>(std::format("/{}/status", kRosLabel), 10);
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
            *command_velocity = Eigen::Vector3d::Zero();
        }
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::Navigation, rmcs_executor::Component)
