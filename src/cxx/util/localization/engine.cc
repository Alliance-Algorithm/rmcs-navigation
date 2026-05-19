#include "cxx/util/localization/engine.hh"
#include "cxx/util/node_mixin.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <rmcs_relocation/srv/relocalize.hpp>

namespace rmcs::navigation {

namespace {

using Relocalize = rmcs_relocation::srv::Relocalize;
using RelocalizeClient = rclcpp::Client<Relocalize>;

auto yaw_to_pose(double x, double y, double yaw) -> geometry_msgs::msg::Pose {
    auto pose = geometry_msgs::msg::Pose{};
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw * 0.5);
    pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
}

auto failed_status(std::string message) -> RelocalizeStatus {
    return RelocalizeStatus{
        .state = RelocalizeState::FAILED,
        .success = false,
        .message = std::move(message),
    };
}

auto status_from_response(const Relocalize::Response& response, bool success) -> RelocalizeStatus {
    const auto& pose = response.estimated_world_base;
    return RelocalizeStatus{
        .state = success ? RelocalizeState::SUCCEEDED : RelocalizeState::FAILED,
        .success = success,
        .message = response.message,
        .fitness_score = response.fitness_score,
        .confidence = response.confidence,
        .estimated_x = pose.position.x,
        .estimated_y = pose.position.y,
        .estimated_z = pose.position.z,
        .estimated_qx = pose.orientation.x,
        .estimated_qy = pose.orientation.y,
        .estimated_qz = pose.orientation.z,
        .estimated_qw = pose.orientation.w,
    };
}

constexpr auto mode_to_msg(RelocalizeMode mode) -> std::uint8_t {
    switch (mode) {
    case RelocalizeMode::Initial: return Relocalize::Request::MODE_INITIAL;
    case RelocalizeMode::Local: return Relocalize::Request::MODE_LOCAL;
    case RelocalizeMode::Wide: return Relocalize::Request::MODE_WIDE;
    }
    return Relocalize::Request::MODE_INITIAL;
}

} // namespace

struct Session {
    mutable std::mutex mutex;
    std::shared_ptr<RelocalizeClient> client;
    rclcpp::Logger logger;
    std::string service_name;

    std::optional<std::int64_t> pending_id;
    RelocalizeStatus last_status{};
    rclcpp::TimerBase::SharedPtr timeout_timer;

    Session(std::shared_ptr<RelocalizeClient> client, rclcpp::Logger logger, std::string name)
        : client{std::move(client)}
        , logger{std::move(logger)}
        , service_name{std::move(name)} {}

    auto begin() -> bool {
        auto lock = std::scoped_lock{mutex};
        if (last_status.state == RelocalizeState::IN_FLIGHT)
            return false;

        pending_id.reset();
        last_status = RelocalizeStatus{
            .state = RelocalizeState::IN_FLIGHT,
            .message = "in_flight",
        };
        return true;
    }

    auto track(std::int64_t id) -> void {
        auto lock = std::scoped_lock{mutex};
        pending_id = id;
    }

    auto end(RelocalizeStatus status, std::optional<std::int64_t> expected_id = std::nullopt)
        -> bool {
        auto lock = std::scoped_lock{mutex};
        if (expected_id && pending_id != *expected_id)
            return false;

        clear_locked();
        last_status = std::move(status);
        return true;
    }

    auto snapshot() const -> RelocalizeStatus {
        auto lock = std::scoped_lock{mutex};
        return last_status;
    }

    auto clear_locked() -> void {
        if (timeout_timer) {
            timeout_timer->cancel();
            timeout_timer.reset();
        }
        if (pending_id) {
            client->remove_pending_request(*pending_id);
            pending_id.reset();
        }
    }
};

struct Localization::Impl : NodeMixin {
    Config config;
    rclcpp::Node& node;
    std::shared_ptr<Session> session;

    explicit Impl(Localization::Config config)
        : config{std::move(config)}
        , node{this->config.rclcpp}
        , session{std::make_shared<Session>(
              node.create_client<Relocalize>(this->config.service_name), node.get_logger(),
              this->config.service_name)} {}

    ~Impl() {
        auto lock = std::scoped_lock{session->mutex};
        session->clear_locked();
    }

    auto get_logger() const -> rclcpp::Logger { return node.get_logger(); }

    auto arm_timeout(
        const std::shared_ptr<Session>& session, const std::shared_ptr<std::int64_t>& pending_id)
        -> void {
        const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>{std::max(0.1, config.request_timeout_sec)});
        auto timer_ref = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
        auto timer = node.create_wall_timer(timeout, [session, pending_id, timer_ref] {
            if (auto timer = timer_ref->lock())
                timer->cancel();

            if (session->end(
                    failed_status("request timeout: " + session->service_name), *pending_id)) {
                RCLCPP_WARN(
                    session->logger, "relocalize request timeout: %s",
                    session->service_name.c_str());
            }
        });
        *timer_ref = timer;

        auto lock = std::scoped_lock{session->mutex};
        if (session->timeout_timer)
            session->timeout_timer->cancel();
        session->timeout_timer = std::move(timer);
    }

    auto send(RelocalizeMode mode, double x, double y, double yaw) -> bool {
        if (!session->begin()) {
            warn("relocalize skipped: previous request is still in flight");
            return false;
        }

        if (!session->client->service_is_ready()
            && !session->client->wait_for_service(std::chrono::seconds{0})) {
            warn("relocalize service unavailable: {}", config.service_name);
            session->end(failed_status("service unavailable: " + config.service_name));
            return false;
        }

        auto request = std::make_shared<Relocalize::Request>();
        request->mode = mode_to_msg(mode);
        request->initial_guess_world_base = yaw_to_pose(x, y, yaw);

        auto pending_id = std::make_shared<std::int64_t>(-1);
        auto session_ref = session;
        auto result = session->client->async_send_request(
            std::move(request), [session_ref, pending_id](RelocalizeClient::SharedFuture future) {
                auto response = future.get();
                if (!response) {
                    RCLCPP_WARN(session_ref->logger, "relocalize response is null");
                    session_ref->end(failed_status("response is null"), *pending_id);
                    return;
                }

                if (!response->success) {
                    RCLCPP_WARN(
                        session_ref->logger, "relocalize failed: %s", response->message.c_str());
                    session_ref->end(status_from_response(*response, false), *pending_id);
                    return;
                }

                RCLCPP_INFO(
                    session_ref->logger, "relocalize success: score=%.4f, confidence=%.3f",
                    response->fitness_score, response->confidence);
                session_ref->end(status_from_response(*response, true), *pending_id);
            });

        *pending_id = result.request_id;
        session->track(result.request_id);
        arm_timeout(session, pending_id);
        return true;
    }
};

Localization::Localization(Config config)
    : pimpl{std::make_unique<Impl>(std::move(config))} {}

Localization::~Localization() noexcept = default;

auto Localization::relocalize(RelocalizeMode mode, double x, double y, double yaw) -> bool {
    return pimpl->send(mode, x, y, yaw);
}

auto Localization::relocalize_status() const -> RelocalizeStatus {
    return pimpl->session->snapshot();
}

} // namespace rmcs::navigation
