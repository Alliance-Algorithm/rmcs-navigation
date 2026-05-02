#pragma once
#include "util/pimpl.hh"

#include "referee/status/field.hpp"

#include <cstdint>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/string.hpp>

#include <expected>
#include <mutex>
#include <string>

namespace rmcs::navigation::details {

struct Context {
    RMCS_PIMPL_DEFINITION(Context)
public:
    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    InputInterface<rmcs_msgs::GameStage> game_stage;
    InputInterface<std::uint16_t> stage_remain_time;
    InputInterface<std::uint64_t> sync_timestamp;
    InputInterface<std::uint8_t> ally_big_energy_activation_status;
    InputInterface<std::uint8_t> ally_small_energy_activation_status;
    InputInterface<std::uint8_t> ally_fortress_occupation_status;
    InputInterface<std::uint8_t> dart_latest_hit_target_total_count;
    InputInterface<rmcs_msgs::RobotId> robot_id;
    InputInterface<std::int64_t> robot_shooter_cooling;
    InputInterface<std::int64_t> robot_shooter_heat_limit;
    InputInterface<double> chassis_power_limit_referee;
    InputInterface<double> chassis_power_referee;
    InputInterface<double> chassis_buffer_energy_referee;
    InputInterface<bool> chassis_output_status;
    InputInterface<rmcs_core::referee::status::GameRobotHp> robots_hp;
    InputInterface<std::uint16_t> ally_hero_hp;
    InputInterface<std::uint16_t> ally_engineer_hp;
    InputInterface<std::uint16_t> ally_infantry_1_hp;
    InputInterface<std::uint16_t> ally_infantry_2_hp;
    InputInterface<std::uint16_t> ally_outpost_hp;
    InputInterface<std::uint16_t> ally_base_hp;
    InputInterface<double> ally_hero_position_x;
    InputInterface<double> ally_hero_position_y;
    InputInterface<double> ally_engineer_position_x;
    InputInterface<double> ally_engineer_position_y;
    InputInterface<double> ally_infantry_1_position_x;
    InputInterface<double> ally_infantry_1_position_y;
    InputInterface<double> ally_infantry_2_position_x;
    InputInterface<double> ally_infantry_2_position_y;
    InputInterface<std::uint16_t> robot_health;
    InputInterface<std::uint16_t> robot_bullet;
    InputInterface<std::uint16_t> robot_42mm_bullet;
    InputInterface<std::uint16_t> robot_fortress_17mm_bullet;
    InputInterface<std::uint16_t> remaining_gold_coin;
    InputInterface<float> robot_initial_speed;
    InputInterface<double> robot_shoot_timestamp;
    InputInterface<bool> sentry_can_confirm_free_revive;
    InputInterface<bool> sentry_can_exchange_instant_revive;
    InputInterface<std::uint16_t> sentry_instant_revive_cost;
    InputInterface<std::uint16_t> sentry_exchanged_bullet;
    InputInterface<std::uint8_t> sentry_remote_bullet_exchange_count;
    InputInterface<std::uint16_t> sentry_exchangeable_bullet;
    InputInterface<std::uint8_t> sentry_mode;
    InputInterface<bool> sentry_energy_mechanism_activatable;
    InputInterface<std::uint32_t> red_score;
    InputInterface<std::uint32_t> blue_score;
    InputInterface<rmcs_msgs::Switch> switch_right;
    InputInterface<rmcs_msgs::Switch> switch_left;

    explicit Context(rclcpp::Node& node, rmcs_executor::Component& component) noexcept;

    auto init(std::mutex& io_mutex, bool mock = false) -> void;
    auto from(const std::string& raw) noexcept -> std::expected<void, std::string>;

    auto health() const noexcept -> std::expected<void, std::string>;
};

} // namespace rmcs::navigation::details
