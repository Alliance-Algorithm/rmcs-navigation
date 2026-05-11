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
        make_input("/referee/game/stage", context.game_stage, mock);
        make_input("/referee/current_hp", context.robot_health, mock);
        make_input("/referee/shooter/bullet_allowance", context.robot_bullet, mock);
        make_input("/referee/id", context.robot_id, mock);

        make_input("/remote/switch/right", context.switch_right, mock);
        make_input("/remote/switch/left", context.switch_left, mock);

        make_input("/tf", context.tf, mock);

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
