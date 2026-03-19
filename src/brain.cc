#include "brain/plan.hh"
#include "util/fsm.hh"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>

namespace rmcs {

constexpr auto kAppLabel = std::string_view{"rmcs-navigation"};
constexpr auto kRosLabel = std::string_view{"rmcs_navigation"};
constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();

class Brain
    : public rmcs_executor::Component
    , public rclcpp::Node {
private:
    /// Rclcpp Interfaces
    ///
    using Twist = geometry_msgs::msg::Twist;
    rclcpp::Subscription<Twist>::SharedPtr subscription_twist;

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

    /// Rmcs Interfaces
    ///
    OutputInterface<Eigen::Vector3d> command_velocity;

    InputInterface<rmcs_msgs::GameStage> game_stage;

    std::chrono::steady_clock::time_point command_received_timestamp;
    std::chrono::milliseconds timeout_interval{100};
    std::atomic<bool> has_warning_timeout = false;

    /// Brain
    ///
    enum class BrainStage {
        WaitForBegin,
        Prepare,
        Running,
        GameOver,
        END,
    };
    Fsm<BrainStage> brain_fsm{BrainStage::WaitForBegin};
    PlanBox plan_box{};

    bool is_fsm_stop = false;

    auto generate_brain_fsm() -> void {
        brain_fsm.use<BrainStage::WaitForBegin>(
            [this] { info("BrainStage: Wait For Begin"); },
            [] {
                // ...
                return BrainStage::Prepare;
            });
        brain_fsm.use<BrainStage::Prepare>(
            [this] { info("BrainStage: Prepare"); },
            [] {
                // ...
                return BrainStage::Running;
            });
        brain_fsm.use<BrainStage::Running>(
            [this] { info("BrainStage: Running"); },
            [] {
                // ...
                return BrainStage::Running;
            });
        brain_fsm.use<BrainStage::GameOver>(
            [this] { info("BrainStage: GameOver"); },
            [] {
                // ...
                return BrainStage::END;
            });

        if (!brain_fsm.fully_registered()) {
            throw std::runtime_error{"The fsm of rmcs_navigation was not fully registered"};
        }
    }

    auto update_goal_position() {}

public:
    explicit Brain()
        : Node{kRosLabel.data()} {

        // RMCS
        const auto name = std::format("/{}/command_velocity", kRosLabel);
        const auto vec_nan = Eigen::Vector3d{kNan, kNan, kNan};
        Component::register_output(name, command_velocity, vec_nan);

        Component::register_input("/referee/game_stage", game_stage, false);

        // NAV2
        subscription_twist = Node::create_subscription<Twist>(
            "/cmd_vel_smoothed", 10, [&, this](const std::unique_ptr<Twist>& msg) {
                command_velocity->x() = msg->linear.x;
                command_velocity->y() = msg->linear.y;
                command_velocity->z() = msg->angular.z;
                command_received_timestamp = std::chrono::steady_clock::now();
                has_warning_timeout = false;
            });
        command_received_timestamp = std::chrono::steady_clock::now();

        // FSM
        generate_brain_fsm();

        // Details
        const auto config_path = ament_index_cpp::get_package_share_directory(kAppLabel.data());
        const auto config_file = std::filesystem::path{config_path} / "config" / "rule.yaml";
        const auto config_yaml = YAML::LoadFile(config_file);

        plan_box.set_rule(config_yaml);
    }

    auto update() -> void override {
        auto update = [this](PlanBox::Information& info) {
            std::ignore = this;
            info.health = 0.0;
            info.health = 0.0;
        };
        plan_box.update_information(update);

        if (!is_fsm_stop) {
            is_fsm_stop = brain_fsm.spin_once();
        }

        using namespace std::chrono_literals;
        if (std::chrono::steady_clock::now() - command_received_timestamp > timeout_interval) {
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
PLUGINLIB_EXPORT_CLASS(rmcs::Brain, rmcs_executor::Component)
