#include "cxx/context.hh"

#include <exception>
#include <type_traits>

#include <yaml-cpp/yaml.h>

namespace rmcs::navigation::details {

namespace {} // namespace

struct Context::Impl {
    Context& context;

    rclcpp::Node& node;
    rmcs_executor::Component& component;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscription;

    std::vector<std::function<std::optional<std::string>()>> healths;

    template <typename T>
    auto make_input(const std::string& name, InputInterface<T>& input, bool mock) -> void {
        if (mock) {
            input.make_and_bind_directly();
        } else {
            component.register_input(name, input, false);
        }

        healths.emplace_back([&, name = name] -> std::optional<std::string> {
            if (input.ready()) {
                return std::nullopt;
            } else {
                return name;
            }
        });
    }

    auto health() const noexcept -> std::expected<void, std::string> {
        auto result = std::string{"Following items are unhealthy\n"};
        if (!std::ranges::all_of(healths, [&](auto& f) {
                auto name = f();
                if (name == std::nullopt)
                    return true;
                result += std::format(" : {}\n", *name);
                return false;
            })) {
            return std::unexpected{result};
        }
        return {};
    }

    template <typename T>
    auto try_sync(InputInterface<T>& input, const YAML::Node& root, const std::string& name)
        -> void {
        if (const auto data = root[name]) {
            auto& to_sync = const_cast<T&>(*input);
            /*^^*/ if constexpr (std::is_enum_v<T>) {
                using U = std::underlying_type_t<T>;
                to_sync = static_cast<T>(data.as<U>());
            } else if constexpr (std::is_constructible_v<T, std::uint8_t>) {
                to_sync = T{data.as<std::uint8_t>()};
            } else {
                to_sync = data.as<T>();
            }
        }
    }

    auto from(const std::string& raw) noexcept -> std::expected<void, std::string> {
        try {
            auto root = YAML::Load(raw);
            if (!root.IsMap())
                return std::unexpected{"context yaml root must be a map"};

            try_sync(context.game_stage, root, "game_stage");
            try_sync(context.robot_health, root, "robot_health");
            try_sync(context.robot_bullet, root, "robot_bullet");
            try_sync(context.red_score, root, "red_score");
            try_sync(context.blue_score, root, "blue_score");

            return {};
        } catch (const std::exception& exception) {
            return std::unexpected{exception.what()};
        }
    }

    auto init(std::mutex& io_mutex, bool mock) {
        make_input("/referee/chassis/power_limit", context.chassis_power_limit_referee, mock);
        make_input("/referee/chassis/power", context.chassis_power_referee, mock);
        make_input("/referee/chassis/buffer_energy", context.chassis_buffer_energy_referee, mock);
        make_input("/referee/chassis/output_status", context.chassis_output_status, mock);

        make_input("/referee/id", context.robot_id, mock);
        make_input("/remote/switch/right", context.switch_right, mock);
        make_input("/remote/switch/left", context.switch_left, mock);
        make_input("/referee/game/stage", context.game_stage, mock);
        make_input("/referee/game/stage_remain_time", context.stage_remain_time, mock);
        make_input("/referee/game/sync_timestamp", context.sync_timestamp, mock);
        make_input(
            "/referee/event/ally_big_energy_activation_status",
            context.ally_big_energy_activation_status, mock);
        make_input(
            "/referee/event/ally_small_energy_activation_status",
            context.ally_small_energy_activation_status, mock);
        make_input(
            "/referee/event/ally_fortress_occupation_status",
            context.ally_fortress_occupation_status, mock);
        make_input(
            "/referee/dart/latest_hit_target_total_count",
            context.dart_latest_hit_target_total_count, mock);
        make_input("/referee/current_hp", context.robot_health, mock);
        make_input("/referee/shooter/bullet_allowance", context.robot_bullet, mock);
        make_input("/referee/shooter/cooling", context.robot_shooter_cooling, mock);
        make_input("/referee/shooter/heat_limit", context.robot_shooter_heat_limit, mock);
        make_input("/referee/shooter/42mm_bullet_allowance", context.robot_42mm_bullet, mock);
        make_input(
            "/referee/shooter/fortress_17mm_bullet_allowance",
            context.robot_fortress_17mm_bullet, mock);
        make_input("/referee/remaining_gold_coin", context.remaining_gold_coin, mock);
        make_input("/referee/shooter/initial_speed", context.robot_initial_speed, mock);
        make_input("/referee/shooter/shoot_timestamp", context.robot_shoot_timestamp, mock);

        make_input("/referee/robots/hp", context.robots_hp, mock);
        make_input("/referee/ally/hero_hp", context.ally_hero_hp, mock);
        make_input("/referee/ally/engineer_hp", context.ally_engineer_hp, mock);
        make_input("/referee/ally/infantry_1_hp", context.ally_infantry_1_hp, mock);
        make_input("/referee/ally/infantry_2_hp", context.ally_infantry_2_hp, mock);
        make_input("/referee/ally/outpost/hp", context.ally_outpost_hp, mock);
        make_input("/referee/ally/base/hp", context.ally_base_hp, mock);
        make_input("/referee/ally/hero_position_x", context.ally_hero_position_x, mock);
        make_input("/referee/ally/hero_position_y", context.ally_hero_position_y, mock);
        make_input("/referee/ally/engineer_position_x", context.ally_engineer_position_x, mock);
        make_input("/referee/ally/engineer_position_y", context.ally_engineer_position_y, mock);
        make_input(
            "/referee/ally/infantry_1_position_x", context.ally_infantry_1_position_x, mock);
        make_input(
            "/referee/ally/infantry_1_position_y", context.ally_infantry_1_position_y, mock);
        make_input(
            "/referee/ally/infantry_2_position_x", context.ally_infantry_2_position_x, mock);
        make_input(
            "/referee/ally/infantry_2_position_y", context.ally_infantry_2_position_y, mock);

        make_input(
            "/referee/sentry/can_confirm_free_revive",
            context.sentry_can_confirm_free_revive, mock);
        make_input(
            "/referee/sentry/can_exchange_instant_revive",
            context.sentry_can_exchange_instant_revive, mock);
        make_input(
            "/referee/sentry/instant_revive_cost", context.sentry_instant_revive_cost, mock);
        make_input(
            "/referee/sentry/exchanged_bullet_allowance",
            context.sentry_exchanged_bullet, mock);
        make_input(
            "/referee/sentry/remote_bullet_exchange_count",
            context.sentry_remote_bullet_exchange_count, mock);
        make_input(
            "/referee/sentry/exchangeable_bullet_allowance",
            context.sentry_exchangeable_bullet, mock);
        make_input("/referee/sentry/mode", context.sentry_mode, mock);
        make_input(
            "/referee/sentry/energy_mechanism_activatable",
            context.sentry_energy_mechanism_activatable, mock);
        make_input("/referee/game/red_score", context.red_score, mock);
        make_input("/referee/game/blue_score", context.blue_score, mock);

        if (mock) {
            constexpr auto topic = "/rmcs_navigation/context/mock";
            subscription = node.create_subscription<std_msgs::msg::String>(
                topic, 10, [&, this](const std::unique_ptr<std_msgs::msg::String>& msg) {
                    auto lock = std::scoped_lock{io_mutex};
                    if (auto result = from(msg->data); !result)
                        RCLCPP_ERROR(
                            node.get_logger(), "Context mock failed: %s", result.error().c_str());
                });
        }
    }
};

Context::Context(rclcpp::Node& node, rmcs_executor::Component& component) noexcept
    : pimpl{std::make_unique<Impl>(*this, node, component)} {}

Context::~Context() noexcept = default;

auto Context::init(std::mutex& io_mutex, bool mock) -> void { pimpl->init(io_mutex, mock); }

auto Context::from(const std::string& raw) noexcept -> std::expected<void, std::string> {
    return pimpl->from(raw);
}

auto Context::health() const noexcept -> std::expected<void, std::string> {
    return pimpl->health();
}

} // namespace rmcs::navigation::details
