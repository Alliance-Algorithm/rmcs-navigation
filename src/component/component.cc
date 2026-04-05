#include "component/util/logger_mixin.hh"
#include "component/util/rmcs_msgs_format.hh" // IWYU pragma: keep

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>

#include <cstdint>
#include <exception>
#include <expected>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sol/sol.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::navigation {

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::LoggerMixin {
private:
    mutable std::mutex io_mutex;

    using Twist = geometry_msgs::msg::Twist;
    std::shared_ptr<rclcpp::Subscription<Twist>> subscription_twist;

    bool mock_context = false;

    std::unique_ptr<sol::state> lua;
    sol::table lua_blackboard;
    sol::protected_function lua_on_init;
    sol::protected_function lua_on_tick;

    struct Context {
        InputInterface<rmcs_msgs::GameStage> game_stage;
        InputInterface<rmcs_msgs::RobotId> robot_id;
        InputInterface<std::uint16_t> robot_health;
        InputInterface<std::uint16_t> robot_bullet;
        InputInterface<std::uint32_t> red_score;
        InputInterface<std::uint32_t> blue_score;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;

        explicit Context(Navigation& self) noexcept
            : self(self) {}

        auto init(bool mock = false) -> void {
            make("/referee/id", robot_id, mock);
            make("/remote/switch/right", switch_right, mock);
            make("/remote/switch/left", switch_left, mock);
            make("/referee/game/stage", game_stage, mock);
            make("/referee/current_hp", robot_health, mock);
            make("/referee/shooter/bullet_allowance", robot_bullet, mock);
            make("/referee/game/red_score", red_score, mock);
            make("/referee/game/blue_score", blue_score, mock);

            if (mock) {
                constexpr auto topic = "/rmcs_navigation/context/mock";
                subscription = self.create_subscription<std_msgs::msg::String>(
                    topic, 10, [this](const std::unique_ptr<std_msgs::msg::String>& msg) {
                        auto lock = std::scoped_lock{self.io_mutex};
                        if (auto result = from(msg->data); !result)
                            self.error("Context mock failed: {}", result.error());
                    });
            }
        }
        auto from(const std::string& raw) noexcept -> std::expected<void, std::string> {
            try {
                auto data = DataHelper{raw};
                if (!data.root.IsMap())
                    return std::unexpected{"context yaml root must be a map"};

                data.try_sync(game_stage, "game_stage");
                data.try_sync(robot_health, "robot_health");
                data.try_sync(robot_bullet, "robot_bullet");
                data.try_sync(red_score, "red_score");
                data.try_sync(blue_score, "blue_score");

                return {};
            } catch (const std::exception& exception) {
                return std::unexpected{exception.what()};
            }
        }

    private:
        struct DataHelper final {
            YAML::Node root;
            explicit DataHelper(const std::string& raw)
                : root{YAML::Load(raw)} {}

            template <typename T>
            auto try_sync(InputInterface<T>& input, const std::string& name) -> void {
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
        };

        Navigation& self;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscription;

        template <typename T>
        auto make(const std::string& name, InputInterface<T>& input, bool mock = false) -> void {
            if (mock)
                input.make_and_bind_directly();
            else
                self.register_input(name, input, true);
        }
    } context;

    struct Command {
        OutputInterface<Eigen::Vector2d> chassis_velocity;
        OutputInterface<std::size_t> nod_count;
        OutputInterface<bool> rotate_chassis;
        OutputInterface<bool> detect_targets;
        OutputInterface<bool> enable_autoaim;

        auto init(Navigation& component) -> void {
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_velocity, Eigen::Vector2d::Zero());
            component.register_output("/rmcs_navigation/nod_count", nod_count, 0);
            component.register_output("/rmcs_navigation/rotate_chassis", rotate_chassis, false);
            component.register_output("/rmcs_navigation/detect_targets", detect_targets, false);
            component.register_output("/rmcs_navigation/start_autoaim", enable_autoaim, false);
        }
    } command;

private:
    static auto rclcpp_option() noexcept {
        return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    }

    auto subscription_twist_callback(const std::unique_ptr<Twist>& msg) {
        auto lock = std::scoped_lock{io_mutex};

        command.chassis_velocity->x() = msg->linear.x;
        command.chassis_velocity->y() = msg->linear.y;
    }

    auto make_api_injection() {
        auto api_result = lua->safe_script("return require('api')", sol::script_pass_on_error);
        if (!api_result.valid()) {
            auto error = api_result.get<sol::error>();
            throw std::runtime_error(std::string{"failed to get lua api: "} + error.what());
        }

        auto api = api_result.get<sol::table>();
        api.set_function("info", [this](const std::string& text) { info("Lua: {}", text); });
        api.set_function("warn", [this](const std::string& text) { warn("Lua: {}", text); });
    }

    auto lua_sync() -> void {
        auto user = lua_blackboard["user"].get<sol::table>();
        user["health"] = *context.robot_health;
        user["bullet"] = *context.robot_bullet;

        auto game = lua_blackboard["game"].get<sol::table>();
        game["stage"] = detail::to_string(*context.game_stage);

        auto play = lua_blackboard["play"].get<sol::table>();
        play["rswitch"] = detail::to_string(*context.switch_right);
        play["lswitch"] = detail::to_string(*context.switch_left);

        auto meta = lua_blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

    auto lua_init() -> void {
        lua = std::make_unique<sol::state>();
        lua->open_libraries(
            sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::os, sol::lib::package,
            sol::lib::string, sol::lib::table);

        // Load Lua Env Path
        auto package_root =
            std::filesystem::path{ament_index_cpp::get_package_share_directory("rmcs-navigation")};
        auto lua_root = package_root / "lua";
        auto package = (*lua)["package"].get<sol::table>();
        auto package_path = package["path"].get_or(std::string{});
        package["path"] = std::format(
            "{};{}/?.lua;{}/?/init.lua", package_path, lua_root.string(), lua_root.string());

        // Api Injection
        make_api_injection();

        // Load Function Binding
        auto load_result = lua->safe_script("require('main')", sol::script_pass_on_error);
        if (!load_result.valid()) {
            auto error = load_result.get<sol::error>();
            throw std::runtime_error(std::string{"failed to load lua main: "} + error.what());
        }

        auto blackboard_result =
            lua->safe_script("return require('blackboard').singleton()", sol::script_pass_on_error);
        if (!blackboard_result.valid()) {
            auto error = blackboard_result.get<sol::error>();
            throw std::runtime_error(std::string{"failed to get lua blackboard: "} + error.what());
        }

        lua_blackboard = blackboard_result.get<sol::table>();
        lua_on_init = (*lua)["on_init"];
        lua_on_tick = (*lua)["on_tick"];

        if (!lua_on_init.valid() || !lua_on_tick.valid()) {
            throw std::runtime_error("lua main must define on_init() and on_tick()");
        }

        // Init Lua First
        auto init_result = lua_on_init.call();
        if (!init_result.valid()) {
            auto error = init_result.get<sol::error>();
            throw std::runtime_error(std::string{"lua on_init failed: "} + error.what());
        }
    }

    auto lua_tick() -> void {
        auto result = lua_on_tick.call();
        if (!result.valid()) {
            auto error = result.get<sol::error>();
            throw std::runtime_error(std::string{"lua on_tick failed: "} + error.what());
        }
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), rclcpp_option()}
        , context{*this} {

        mock_context = get_parameter_or("mock_context", false);

        context.init(mock_context);
        command.init(*this);

        lua_init();

        const auto command_vel_name = get_parameter("command_vel_name").as_string();
        subscription_twist = Node::create_subscription<Twist>(
            command_vel_name, 10,
            [this](const std::unique_ptr<Twist>& msg) { subscription_twist_callback(msg); });
    }

    auto update() -> void override {
        auto lock = std::scoped_lock{io_mutex};
        lua_sync();
        lua_tick();
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
