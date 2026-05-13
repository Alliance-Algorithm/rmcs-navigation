#include "cxx/context.hh"
#include "cxx/controller/normal.hh"
#include "cxx/lua_context.hh"
#include "cxx/navigation.hh"
#include "cxx/util/node_mixin.hh"

#include <chrono>
#include <cmath>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/rmcs_msgs.hpp> // IWYU pragma: keep

namespace rmcs::navigation {

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs::navigation::NodeMixin {
private:
    static constexpr auto kCmdVelTimeout = std::chrono::milliseconds{500};

    mutable std::mutex io_mutex;

    bool mock_context = false;

    std::atomic<std::uint16_t> lua_tick_count = 0;

    details::LuaContext lua_context{*this};
    details::Navigation navigation{*this};
    details::Context context{*this, *this};

    // 控制器框架
    IController* selected_controller = nullptr;
    std::unordered_map<std::string, std::unique_ptr<IController>> controllers;

    Eigen::Vector2d desired_direction = Eigen::Vector2d::Zero();
    double current_world_yaw = std::numeric_limits<double>::quiet_NaN();

    struct Command {
        using ChassisMode = rmcs_msgs::ChassisMode;
        using Context = details::Context;

        enum class PendingKind {
            None,
            BuyBullet,
            SwitchMode,
            ConfirmRevive,
        };

        struct PendingDecision {
            PendingKind kind = PendingKind::None;
            std::chrono::steady_clock::time_point deadline =
                std::chrono::steady_clock::time_point::min();
            std::uint16_t target_bullet = 0;
            std::uint8_t target_mode = 0;
            bool expected_confirm_revive_consumed = false;
        };

        OutputInterface<bool> enable_control;
        OutputInterface<bool> enable_autoaim;
        OutputInterface<ChassisMode> chassis_behavior;
        OutputInterface<Eigen::Vector2d> chassis_speed;
        OutputInterface<Eigen::Vector2d> gimbal_speed;
        OutputInterface<bool> sentry_decision_enabled;
        OutputInterface<std::uint16_t> sentry_bullet_exchange_value;
        OutputInterface<std::uint8_t> requested_mode;
        OutputInterface<bool> sentry_confirm_revive;

        InputInterface<uint16_t> sentry_exchanged_bullet_;
        // InputInterface<int> sentry_mode_now ;
        PendingDecision pending_decision;

        auto init(Navigation& component) -> void {
            component.register_output("/rmcs_navigation/enable_control", enable_control, false);
            component.register_output("/rmcs_navigation/enable_autoaim", enable_autoaim, false);
            component.register_output(
                "/rmcs_navigation/chassis_behavior", chassis_behavior, ChassisMode::AUTO);
            component.register_output(
                "/rmcs_navigation/chassis_velocity", chassis_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/rmcs_navigation/gimbal_velocity", gimbal_speed, Eigen::Vector2d::Zero());
            component.register_output(
                "/referee/sentry/decision/enabled", sentry_decision_enabled, false);
            component.register_output(
                "/referee/sentry/decision/bullet_exchange_value", sentry_bullet_exchange_value,
                std::uint16_t{0});
            component.register_output(
                "/referee/sentry/decision/mode", requested_mode, std::uint8_t{0});
            component.register_output(
                "/referee/sentry/decision/confirm_revive", sentry_confirm_revive, false);

            // component.register_input("/referee/sentry/mode", sentry_mode_now, 3);
            component.register_input("/referee/sentry/exchanged_bullet_allowance", sentry_exchanged_bullet_, 0);
        }

        auto exchange_17mm_bullet(int amount) -> void {
            constexpr int max_bullet_exchange_value = 0x07ff;
            amount = std::clamp(amount, 0, max_bullet_exchange_value);

            if (amount == 0) {
                clear_sentry_decision();
                return;
            }

            reset_all_decisions();
            *sentry_bullet_exchange_value = static_cast<std::uint16_t>(amount);
            // *requested_mode = static_cast<std::uint8_t>(*sentry_mode_now);
            activate_decision(
                PendingDecision{
                    .kind = PendingKind::BuyBullet,
                    .target_bullet = static_cast<std::uint16_t>(amount),
                });
        }

        auto switch_mode(int mode) -> void {
            if (mode < 1 || mode > 3)
                return;

            reset_all_decisions();
            *requested_mode = static_cast<std::uint8_t>(mode);
            *sentry_bullet_exchange_value = static_cast<std::uint16_t>(*sentry_exchanged_bullet_);
            activate_decision(
                PendingDecision{
                    .kind = PendingKind::SwitchMode,
                    .target_bullet = static_cast<std::uint16_t>(*sentry_exchanged_bullet_),
                    .target_mode = static_cast<std::uint8_t>(mode),
                });
        }

        auto confirm_revive(const Context& context) -> void {
            if (!context.sentry_can_confirm_free_revive.ready()
                || !*context.sentry_can_confirm_free_revive) {
                return;
            }
            *sentry_bullet_exchange_value = static_cast<std::uint16_t>(*sentry_exchanged_bullet_);

            reset_all_decisions();
            *sentry_confirm_revive = true;
            activate_decision(
                PendingDecision{
                    .kind = PendingKind::ConfirmRevive,
                    .target_bullet = static_cast<std::uint16_t>(*sentry_exchanged_bullet_),
                    .expected_confirm_revive_consumed = true,
                    
                });
        }

        auto update(const Context& context) -> void {
            if (pending_decision.kind == PendingKind::None)
                return;

            if (decision_satisfied(context) || std::chrono::steady_clock::now() >= pending_decision.deadline)
                clear_sentry_decision();
        }

    private:
        auto decision_satisfied(const Context& context) const -> bool {
            switch (pending_decision.kind) {
            case PendingKind::None: return true;
            case PendingKind::BuyBullet:
                return context.sentry_exchanged_bullet.ready()
                    && *context.sentry_exchanged_bullet >= pending_decision.target_bullet;
            case PendingKind::SwitchMode:
                return context.sentry_mode.ready()
                    && *context.sentry_mode == pending_decision.target_mode;
            case PendingKind::ConfirmRevive:
                return pending_decision.expected_confirm_revive_consumed
                    && context.sentry_can_confirm_free_revive.ready()
                    && !*context.sentry_can_confirm_free_revive;
            }
            return false;
        }

        auto reset_all_decisions() -> void {
            *sentry_bullet_exchange_value = std::uint16_t{0};
            *requested_mode = std::uint8_t{0};
            *sentry_confirm_revive = false;
        }

        auto activate_decision(PendingDecision decision) -> void {
            *sentry_decision_enabled = true;
            decision.deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
            pending_decision = decision;
        }

        auto clear_sentry_decision() -> void {
            *sentry_decision_enabled = false;
            reset_all_decisions();
            pending_decision = PendingDecision{};
        }
    } command;

private:
    static auto option() noexcept {
        return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    }

    auto sync_blackboard() {
        const auto [x, y, yaw] = navigation.check_position();
        current_world_yaw = yaw; // 高频查询 TF 是不对的，所以应该先缓存一份

        auto& blackboard = lua_context.blackboard();

        auto read_context =
            []<typename T>(const details::Context::InputInterface<T>& input, T fallback) -> T {
            if (input.ready())
                return *input;
            return fallback;
        };

        auto user = blackboard["user"].get<sol::table>();
        user["health"] = read_context(context.robot_health, std::uint16_t{0});
        user["bullet"] = read_context(context.robot_bullet, std::uint16_t{0});
        user["chassis_power_limit"] = read_context(context.chassis_power_limit_referee, 0.0);
        user["chassis_power"] = read_context(context.chassis_power_referee, 0.0);
        user["chassis_buffer_energy"] = read_context(context.chassis_buffer_energy_referee, 0.0);
        user["chassis_output_status"] = read_context(context.chassis_output_status, false);
        user["shooter_cooling"] = read_context(context.robot_shooter_cooling, std::int64_t{0});
        user["shooter_heat_limit"] =
            read_context(context.robot_shooter_heat_limit, std::int64_t{0});
        user["bullet_42mm"] = read_context(context.robot_42mm_bullet, std::uint16_t{0});
        user["fortress_17mm_bullet"] =
            read_context(context.robot_fortress_17mm_bullet, std::uint16_t{0});
        user["initial_speed"] = read_context(context.robot_initial_speed, 0.0F);
        user["shoot_timestamp"] = read_context(context.robot_shoot_timestamp, 0.0);
        user["x"] = x;
        user["y"] = y;
        user["yaw"] = yaw;

        auto game = blackboard["game"].get<sol::table>();
        game["stage"] =
            rmcs_msgs::to_string(read_context(context.game_stage, rmcs_msgs::GameStage::UNKNOWN));
        game["sync_timestamp"] = read_context(context.sync_timestamp, std::uint64_t{0});
        game["outpost_health"] = read_context(context.ally_outpost_hp, std::uint16_t{0});
        game["base_health"] = read_context(context.ally_base_hp, std::uint16_t{0});
        game["hero_health"] = read_context(context.ally_hero_hp, std::uint16_t{0});
        game["infantry_1_health"] = read_context(context.ally_infantry_1_hp, std::uint16_t{0});
        game["infantry_2_health"] = read_context(context.ally_infantry_2_hp, std::uint16_t{0});
        game["engineer_health"] = read_context(context.ally_engineer_hp, std::uint16_t{0});
        game["remaining_time"] = read_context(context.stage_remain_time, std::uint16_t{0});
        game["gold_coin"] = read_context(context.remaining_gold_coin, std::uint16_t{0});
        game["exchangeable_ammunition_quantity"] =
            read_context(context.sentry_exchangeable_bullet, std::uint16_t{0});
        const auto our_dart_number_of_hits = static_cast<int>(
            read_context(context.dart_latest_hit_target_total_count, std::uint8_t{0}));
        game["our_dart_number_of_hits"] = our_dart_number_of_hits;
        game["our_dart_nmber_of_hits"] = our_dart_number_of_hits;
        game["fortress_occupied"] =
            read_context(context.ally_fortress_occupation_status, std::uint8_t{0}) != 0;
        game["big_energy_mechanism_activated"] =
            read_context(context.ally_big_energy_activation_status, std::uint8_t{0}) != 0;
        game["small_energy_mechanism_activated"] =
            read_context(context.ally_small_energy_activation_status, std::uint8_t{0}) != 0;

        auto set_position = [](sol::table position, double px, double py) {
            position["x"] = px;
            position["y"] = py;
        };
        set_position(
            game["hero_position"].get<sol::table>(),
            read_context(context.ally_hero_position_x, 0.0),
            read_context(context.ally_hero_position_y, 0.0));
        set_position(
            game["infantry_1_position"].get<sol::table>(),
            read_context(context.ally_infantry_1_position_x, 0.0),
            read_context(context.ally_infantry_1_position_y, 0.0));
        set_position(
            game["infantry_2_position"].get<sol::table>(),
            read_context(context.ally_infantry_2_position_x, 0.0),
            read_context(context.ally_infantry_2_position_y, 0.0));
        set_position(
            game["engineer_position"].get<sol::table>(),
            read_context(context.ally_engineer_position_x, 0.0),
            read_context(context.ally_engineer_position_y, 0.0));

        game["robot_id"] = static_cast<int>(
            read_context(context.robot_id, rmcs_msgs::RobotId{rmcs_msgs::RobotId::UNKNOWN}));
        game["can_confirm_free_revive"] =
            read_context(context.sentry_can_confirm_free_revive, false);
        game["can_exchange_instant_revive"] =
            read_context(context.sentry_can_exchange_instant_revive, false);
        game["instant_revive_cost"] =
            read_context(context.sentry_instant_revive_cost, std::uint16_t{0});
        game["exchanged_bullet"] =
            read_context(context.sentry_exchanged_bullet, std::uint16_t{0});
        game["remote_bullet_exchange_count"] =
            read_context(context.sentry_remote_bullet_exchange_count, std::uint8_t{0});
        game["sentry_mode"] = read_context(context.sentry_mode, std::uint8_t{0});
        game["energy_mechanism_activatable"] =
            read_context(context.sentry_energy_mechanism_activatable, false);

        auto play = blackboard["play"].get<sol::table>();
        play["rswitch"] =
            rmcs_msgs::to_string(read_context(context.switch_right, rmcs_msgs::Switch::UNKNOWN));
        play["lswitch"] =
            rmcs_msgs::to_string(read_context(context.switch_left, rmcs_msgs::Switch::UNKNOWN));

        auto map_command = blackboard["map_command"].get<sol::table>();
        map_command["x"] = read_context(context.map_command_event_x, 0.0);
        map_command["y"] = read_context(context.map_command_event_y, 0.0);
        map_command["keyboard"] =
            static_cast<int>(read_context(context.map_command_event_keyboard, std::uint8_t{0}));
        map_command["target_robot_id"] = static_cast<int>(
            read_context(context.map_command_event_target_robot_id, std::uint8_t{0}));
        map_command["source"] =
            static_cast<int>(read_context(context.map_command_event_source, std::uint16_t{0}));
        map_command["sequence"] =
            read_context(context.map_command_event_sequence, std::uint64_t{0});

        auto meta = blackboard["meta"].get<sol::table>();
        meta["timestamp"] = this->now().seconds();
    }

public:
    explicit Navigation()
        : rclcpp::Node{get_component_name(), option()} {

        mock_context = param<bool>("mock_context");

        context.init(io_mutex, mock_context);
        command.init(*this);

        lua_context.init({
            .update_enable_control = [this](bool enable) { *command.enable_control = enable; },
            .send_target = [this](double x, double y) { navigation.send_target(x, y); },
            .switch_topic_forward =
                [this](bool enable) { navigation.switch_topic_forward(enable); },
            .update_gimbal_direction = [this](double angle) { desired_direction = {angle, 0.0}; },
            .switch_controller =
                [this](const std::string& mode) {
                    if (!controllers.contains(mode)) {
                        selected_controller = nullptr;
                        logging::fuck("controller '{}' not found", mode);
                        return;
                    }
                    selected_controller = controllers.at(mode).get();
                    logging::info("switched to controller '{}'", mode);
                },
            .exchange_17mm_bullet = [this](int amount) { command.exchange_17mm_bullet(amount); },
            .switch_mode = [this](int mode) { command.switch_mode(mode); },
            .confirm_revive = [this]() { command.confirm_revive(context); },
        });

        controllers["normal"] = std::make_unique<NormalController>();
        selected_controller = controllers["normal"].get();

        logging::info("Navigation is initialized");
    }

    auto before_updating() -> void override {
        if (auto ok = context.health(); !ok) {
            logging::fuck("{}", ok.error());
            throw std::runtime_error{"Context Error"};
        }
    }

    auto update() -> void override {
        command.update(context);

        if (lua_tick_count++ == 100) [[unlikely]] {
            lua_tick_count = 0;
            auto lock = std::scoped_lock{io_mutex};
            sync_blackboard();
            lua_context.tick();
        }

        if (selected_controller && *command.enable_control) {
            const auto nav_cmd = navigation.current_command();
            const auto elapsed = std::chrono::steady_clock::now() - nav_cmd.timestamp;
            const auto effective_vel =
                (elapsed > kCmdVelTimeout) ? Eigen::Vector2d::Zero() : nav_cmd.speed;

            const auto direction = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::BottomYawLink::DirectionVector{Eigen::Vector3d::UnitX()},
                *context.tf);
            auto vector = *direction;
            vector.z() = 0.0;
            if (vector.norm() > 1e-9)
                vector.normalize();
            else
                vector = Eigen::Vector3d::UnitX();
            const auto current_local_yaw = std::atan2(vector.y(), vector.x());

            selected_controller->update_context({
                .target_chassis_speed = effective_vel,
                .target_gimbal_toward = desired_direction,
                .current_local_yaw = current_local_yaw,
                .current_world_yaw = current_world_yaw,
            });

            auto cmd = selected_controller->generate_command();
            *command.chassis_speed = cmd.chassis_speed;
            *command.gimbal_speed = cmd.gimbal_toward;
            *command.chassis_behavior = cmd.chassis_mode;
        } else {
            *command.chassis_speed = Eigen::Vector2d::Zero();
            *command.gimbal_speed = Eigen::Vector2d::Zero();
        }
    }
};

} // namespace rmcs::navigation

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::navigation::Navigation, rmcs_executor::Component)
