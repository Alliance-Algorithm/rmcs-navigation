#include "navigation.hh"
#include "util/node_mixin.hh"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
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

namespace {

auto parse_pose_yaw(const geometry_msgs::msg::Quaternion& rotation) -> double {
    const auto sin_yaw = 2.0 * ((rotation.w * rotation.z) + (rotation.x * rotation.y));
    const auto cos_yaw = 1.0 - (2.0 * ((rotation.y * rotation.y) + (rotation.z * rotation.z)));
    return std::atan2(sin_yaw, cos_yaw);
}

auto make_yaw_quaternion(double yaw) -> geometry_msgs::msg::Quaternion {
    auto orientation = geometry_msgs::msg::Quaternion{};
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
    return orientation;
}

} // namespace

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
    static constexpr auto kYawEpsilon = 1e-2;
    static constexpr auto kServerWaitTimeout = std::chrono::seconds{1};
    static constexpr auto kNavigateToPoseActionName = "/navigate_to_pose";
    static constexpr auto kWorldFrame = "world";
    static constexpr auto kBaseFrame = "base_link";
    static constexpr auto kMoveBaseGoalTopic = "/move_base_simple/goal";
    static constexpr auto kGoalPoseTopic = "/goal_pose";

    struct Target final {
        double x = std::numeric_limits<double>::quiet_NaN();
        double y = std::numeric_limits<double>::quiet_NaN();
        double yaw = std::numeric_limits<double>::quiet_NaN();

        static auto same_yaw(double lhs, double rhs) noexcept -> bool {
            const auto lhs_finite = std::isfinite(lhs);
            const auto rhs_finite = std::isfinite(rhs);
            if (!lhs_finite || !rhs_finite)
                return lhs_finite == rhs_finite;

            return std::abs(std::remainder(lhs - rhs, 2 * std::numbers::pi_v<double>))
                <= Impl::kYawEpsilon;
        }

        auto operator==(const Target& rhs) const noexcept -> bool {
            return std::abs(x - rhs.x) <= Impl::kPositionEpsilon
                && std::abs(y - rhs.y) <= Impl::kPositionEpsilon && same_yaw(yaw, rhs.yaw);
        }
    };

    rclcpp::Node& node;
    mutable std::mutex goal_mutex;

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
        auto lock = std::scoped_lock{goal_mutex};
        return active_goal.has_value() && *active_goal == goal;
    }

    auto set_active_goal(const Target& goal) -> void {
        auto lock = std::scoped_lock{goal_mutex};
        active_goal = goal;
    }

    auto reset_active_goal() -> void {
        auto lock = std::scoped_lock{goal_mutex};
        active_goal.reset();
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
        navigation_goal_message.pose.pose.orientation =
            std::isfinite(goal.yaw) ? make_yaw_quaternion(goal.yaw) : make_yaw_quaternion(0.0);
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
                        reset_active_goal();

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
            };
        options.result_callback = [this, goal, request_id](const GoalResult& result) {
            if (request_id != latest_request_id)
                return;

            reset_active_goal();
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
                auto yaw = parse_pose_yaw(message->pose.orientation);
                send_target(position.x, position.y, yaw);
                info(
                    "forward {} -> ({:.1}, {:.1}, yaw={:.2f}rad)", topic, position.x, position.y,
                    yaw);
            });
    }

public:
    explicit Impl(rclcpp::Node& node)
        : node{node} {}

    ~Impl() {
        auto current_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});
        reset_active_goal();
        ++latest_request_id;

        if (current_handle)
            client->async_cancel_goal(current_handle);
    }

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

    auto send_target(double x, double y) -> void {
        send_target(x, y, std::numeric_limits<double>::quiet_NaN());
    }

    auto send_target(double x, double y, double yaw) -> void {
        auto goal = Target{.x = x, .y = y, .yaw = yaw};
        if (has_same_goal(goal))
            return;

        if (!ensure_server_ready())
            return;

        if (has_same_goal(goal))
            return;

        set_active_goal(goal);
        auto request_id = ++latest_request_id;
        auto cancel_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});

        if (cancel_handle)
            client->async_cancel_goal(cancel_handle);

        send_target(goal, request_id);
    }

    auto cancel_target() -> void {
        auto cancel_handle = std::exchange(current_goal_handle, std::shared_ptr<GoalHandle>{});
        reset_active_goal();
        ++latest_request_id;

        if (cancel_handle)
            client->async_cancel_goal(cancel_handle);
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
            auto yaw = parse_pose_yaw(transform.transform.rotation);

            return {translation.x, translation.y, yaw};
        } catch (const tf2::TransformException&) {
            constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
            return {kNan, kNan, kNan};
        }
    }

    auto check_bottom_yaw() const -> double { return std::get<2>(check_position()); }

    auto check_active_goal() const -> std::tuple<double, double, double> {
        auto lock = std::scoped_lock{goal_mutex};
        if (!active_goal.has_value()) {
            constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
            return {kNan, kNan, kNan};
        }

        return {active_goal->x, active_goal->y, active_goal->yaw};
    }
};

Navigation::Navigation(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(node)} {}

Navigation::~Navigation() noexcept = default;

auto Navigation::send_target(double x, double y) -> void { pimpl->send_target(x, y); }
auto Navigation::send_target(double x, double y, double yaw) -> void {
    pimpl->send_target(x, y, yaw);
}

auto Navigation::switch_topic_forward(bool enable) -> void { pimpl->switch_topic_forward(enable); }

auto Navigation::check_position() const -> std::tuple<double, double, double> {
    return pimpl->check_position();
}

auto Navigation::check_bottom_yaw() const -> double { return pimpl->check_bottom_yaw(); }

auto Navigation::check_active_goal() const -> std::tuple<double, double, double> {
    return pimpl->check_active_goal();
}

auto Navigation::cancel_target() -> void { pimpl->cancel_target(); }

} // namespace rmcs::navigation::details
