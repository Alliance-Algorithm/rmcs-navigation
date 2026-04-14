#include "navigation.hh"
#include "util/node_mixin.hh"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rmcs::navigation::details {

struct Navigation::Impl : rmcs::navigation::NodeMixin {

    using GoalAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<GoalAction>;
    using GoalResult = GoalHandle::WrappedResult;
    using GoalClient = rclcpp_action::Client<GoalAction>;
    using GoalSubscription = rclcpp::Subscription<geometry_msgs::msg::PoseStamped>;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    using TfBuffer = tf2_ros::Buffer;
    using TfListener = tf2_ros::TransformListener;

    static constexpr auto kPositionEpsilon = 1e-2;
    static constexpr auto kServerWaitTimeout = std::chrono::seconds{1};
    static constexpr auto kNavigateToPoseActionName = "/navigate_to_pose";
    static constexpr auto kWorldFrame = "world";
    static constexpr auto kBaseFrame = "base_link";
    static constexpr auto kMoveBaseGoalTopic = "/move_base_simple/goal";
    static constexpr auto kGoalPoseTopic = "/goal_pose";

    struct Target final {
        double x = std::numeric_limits<double>::quiet_NaN();
        double y = std::numeric_limits<double>::quiet_NaN();

        auto operator==(const Target& rhs) const noexcept -> bool {
            return std::abs(x - rhs.x) <= Impl::kPositionEpsilon
                && std::abs(y - rhs.y) <= Impl::kPositionEpsilon;
        }
    };

    rclcpp::Node& node;

    // TF Query
    std::shared_ptr<TfBuffer> tf_buffer = std::make_shared<TfBuffer>(node.get_clock());
    std::shared_ptr<TfListener> tf_listener =
        std::make_shared<TfListener>(*tf_buffer, &node, false);

    // Navigation Action
    std::shared_ptr<GoalClient> client =
        rclcpp_action::create_client<GoalAction>(&node, kNavigateToPoseActionName);
    GoalAction::Goal navigation_goal_message = [] {
        auto message = GoalAction::Goal{};
        message.pose.header.frame_id = kWorldFrame;
        message.pose.pose.orientation.w = 1.0;
        message.pose.pose.orientation.x = 0.0;
        message.pose.pose.orientation.y = 0.0;
        message.pose.pose.orientation.z = 0.0;
        return message;
    }();

    // Topic Forwarding (Optional)
    std::shared_ptr<GoalSubscription> move_base_goal_subscription;
    std::shared_ptr<GoalSubscription> goal_pose_subscription;
    bool topic_forward_enabled = false;

    // Goal Runtime State
    std::shared_ptr<GoalHandle> current_goal_handle;
    std::optional<Target> active_goal;

    std::uint64_t latest_request_id = 0;

    auto has_same_goal(const Target& goal) const -> bool {
        return active_goal.has_value() && *active_goal == goal;
    }

    auto ensure_server_ready() -> bool {
        if (client->action_server_is_ready())
            return true;

        if (client->wait_for_action_server(kServerWaitTimeout))
            return true;

        warn("{} action server is not available", kNavigateToPoseActionName);
        return false;
    }

    auto update_target_message(const Target& goal) -> void {
        navigation_goal_message.pose.header.stamp = node.now();
        navigation_goal_message.pose.pose.position.x = goal.x;
        navigation_goal_message.pose.pose.position.y = goal.y;
    }

    auto log_result(const Target& goal, rclcpp_action::ResultCode code) const -> void {
        switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            info("navigation goal reached: x={}, y={}", goal.x, goal.y);
            return;
        case rclcpp_action::ResultCode::ABORTED:
            warn("navigation goal aborted: x={}, y={}", goal.x, goal.y);
            return;
        case rclcpp_action::ResultCode::CANCELED:
            info("navigation goal canceled: x={}, y={}", goal.x, goal.y);
            return;
        default:
            warn("navigation goal finished with unknown status: x={}, y={}", goal.x, goal.y);
            return;
        }
    }

    auto send_target(const Target& goal, std::uint64_t request_id) -> void {
        update_target_message(goal);

        auto options = GoalClient::SendGoalOptions{};
        options.goal_response_callback = //
            [request_id, this, goal](const std::shared_ptr<GoalHandle>& handle) {
                if (!handle) {
                    if (request_id == latest_request_id)
                        active_goal.reset();

                    warn("navigate_to_pose goal rejected: x={}, y={}", goal.x, goal.y);
                    return;
                }

                auto should_cancel = request_id != latest_request_id;
                if (!should_cancel)
                    current_goal_handle = handle;

                if (should_cancel) {
                    client->async_cancel_goal(handle);
                    return;
                }

                info("send navigation goal: x={}, y={}", goal.x, goal.y);
            };
        options.result_callback = [this, goal, request_id](const GoalResult& result) {
            if (request_id != latest_request_id)
                return;

            active_goal.reset();
            current_goal_handle.reset();
            log_result(goal, result.code);
        };

        client->async_send_goal(navigation_goal_message, options);
    }

    auto make_topic_forward_subscription(std::string_view topic) {
        return node.create_subscription<PoseStamped>(
            std::string{topic}, 10,
            [this, topic = std::string{topic}](const std::unique_ptr<PoseStamped>& message) {
                if (!message)
                    return;

                auto& position = message->pose.position;
                send_target(position.x, position.y);
                info("forward goal from {}: x={}, y={}", topic, position.x, position.y);
            });
    }

public:
    explicit Impl(rclcpp::Node& node_ref)
        : node{node_ref} {}

    ~Impl() {
        auto current_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});
        active_goal.reset();
        ++latest_request_id;

        if (current_handle)
            client->async_cancel_goal(current_handle);
    }

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

    auto send_target(double x, double y) -> void {
        auto goal = Target{.x = x, .y = y};
        if (has_same_goal(goal))
            return;

        if (!ensure_server_ready())
            return;

        if (has_same_goal(goal))
            return;

        active_goal = goal;
        auto request_id = ++latest_request_id;
        auto cancel_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});

        if (cancel_handle) {
            info("replace active navigation goal with x={}, y={}", goal.x, goal.y);
            client->async_cancel_goal(cancel_handle);
        }

        send_target(goal, request_id);
    }

    auto switch_topic_forward(bool enable) -> void {
        if (topic_forward_enabled == enable)
            return;

        topic_forward_enabled = enable;
        if (topic_forward_enabled) {
            move_base_goal_subscription = make_topic_forward_subscription(kMoveBaseGoalTopic);
            goal_pose_subscription = make_topic_forward_subscription(kGoalPoseTopic);
            info("goal topic forwarding enabled");
            return;
        }

        move_base_goal_subscription.reset();
        goal_pose_subscription.reset();
        info("goal topic forwarding disabled");
    }

    auto check_position() const -> std::tuple<double, double, double> {
        try {
            auto transform =
                tf_buffer->lookupTransform(kWorldFrame, kBaseFrame, tf2::TimePointZero);
            auto& translation = transform.transform.translation;
            auto& rotation = transform.transform.rotation;

            auto sin_yaw = 2.0 * ((rotation.w * rotation.z) + (rotation.x * rotation.y));
            auto cos_yaw = 1.0 - (2.0 * ((rotation.y * rotation.y) + (rotation.z * rotation.z)));
            auto yaw = std::atan2(sin_yaw, cos_yaw);

            return {translation.x, translation.y, yaw};
        } catch (const tf2::TransformException& exception) {
            constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
            return {kNan, kNan, kNan};
        }
    }
};

Navigation::Navigation(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(node)} {}

Navigation::~Navigation() noexcept = default;

auto Navigation::send_target(double x, double y) -> void { pimpl->send_target(x, y); }

auto Navigation::switch_topic_forward(bool enable) -> void { pimpl->switch_topic_forward(enable); }

auto Navigation::check_position() const -> std::tuple<double, double, double> {
    return pimpl->check_position();
}

} // namespace rmcs::navigation::details
