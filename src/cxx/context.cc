#include "cxx/context.hh"
#include "cxx/util/service.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace rmcs::navigation::details {

struct RmcsContext::Impl {
    RmcsContext& context;
    rclcpp::Node& node;
    rmcs_executor::Component& component;

    std::vector<std::function<void()>> default_binders;
    std::vector<std::function<std::optional<std::string>()>> readiness_checks;

    std::atomic<bool> stop_service{false};
    std::thread service_thread;

    template <typename T>
    auto make_input(const std::string& name, InputInterface<T>& input, T default_value) -> void {
        component.register_input(name, input, false);

        readiness_checks.emplace_back([&, name]() -> std::optional<std::string> {
            if (input.ready())
                return std::nullopt;
            return name;
        });

        default_binders.emplace_back([&, default_value = std::move(default_value)]() mutable {
            if (!input.ready()) {
                input.make_and_bind_directly(std::move(default_value));
            }
        });
    }

    auto ensure_defaults() -> void {
        for (const auto& check : readiness_checks) {
            if (auto name = check()) {
                RCLCPP_WARN(
                    node.get_logger(), "Context input not ready, binding default: %s",
                    name->c_str());
            }
        }
        for (auto& binder : default_binders) {
            binder();
        }
    }

    template <typename T>
    static auto write_input(InputInterface<T>& input, T value) -> void {
        if (!input.ready())
            return;
        const_cast<T&>(*input) = std::move(value);
    }

    auto init() {
        make_input("/auto_aim/should_control", context.auto_aim_should_control, false);

        make_input("/referee/chassis/power_limit", context.chassis_power_limit_referee, 100.0);
        make_input("/chassis/climber/status", context.climber_status, 0.0);
        make_input("/referee/game/stage", context.game_stage, rmcs_msgs::GameStage::UNKNOWN);
        make_input("/referee/current_hp", context.robot_health, std::uint16_t{400});
        make_input("/referee/shooter/bullet_allowance", context.robot_bullet, std::uint16_t{300});
        make_input("/referee/id", context.robot_id, rmcs_msgs::RobotId{});
        make_input("/referee/enemy/outpost/hp", context.enemy_outpost_hp, std::uint16_t{0});
        make_input("/referee/enemy/base/hp", context.enemy_base_hp, std::uint16_t{0});

        make_input("/remote/switch/right", context.switch_right, rmcs_msgs::Switch::UNKNOWN);
        make_input("/remote/switch/left", context.switch_left, rmcs_msgs::Switch::UNKNOWN);
        make_input("/remote/joystick/right", context.rjoystick, Eigen::Vector2d{0.0, 0.0});
        make_input("/remote/joystick/left", context.ljoystick, Eigen::Vector2d{0.0, 0.0});

        make_input("/tf", context.tf, rmcs_description::SentryTf{});
        make_input("/auto_aim/robot_center", context.enemy_center, Eigen::Vector3d{0.0, 0.0, 0.0});

        service_thread = std::thread{[this] {
            auto service = util::make_service<"context">(
                util::make_action<"game_stage", int>([this](int value) {
                    write_input(context.game_stage, static_cast<rmcs_msgs::GameStage>(value));
                }),
                util::make_action<"robot_health", int>([this](int value) {
                    write_input(context.robot_health, static_cast<std::uint16_t>(value));
                }),
                util::make_action<"robot_bullet", int>([this](int value) {
                    write_input(context.robot_bullet, static_cast<std::uint16_t>(value));
                }));

            while (!stop_service.load(std::memory_order::relaxed)) {
                service.spin_once();
                std::this_thread::sleep_for(std::chrono::milliseconds{100});
            }
        }};
        RCLCPP_INFO(node.get_logger(), "Context service at /tmp/rmcs-navigation/context/");
    }

    ~Impl() {
        stop_service.store(true, std::memory_order::relaxed);
        if (service_thread.joinable())
            service_thread.join();
    }
};

RmcsContext::RmcsContext(rclcpp::Node& node, rmcs_executor::Component& component) noexcept
    : pimpl{std::make_unique<Impl>(*this, node, component)} {}

RmcsContext::~RmcsContext() noexcept = default;

auto RmcsContext::init() -> void { pimpl->init(); }

auto RmcsContext::ensure_defaults() -> void { pimpl->ensure_defaults(); }

} // namespace rmcs::navigation::details
