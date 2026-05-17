#pragma once
#include "cxx/util/fsm.hh"
#include "cxx/util/math/low_pass.hh"
#include "cxx/util/node_mixin.hh"

#include <chrono>
#include <cmath>
#include <limits>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <rmcs_msgs/chassis_mode.hpp>

namespace rmcs::navigation {

class MotionFsm {
    using ChassisMode = rmcs_msgs::ChassisMode;
    using clock = std::chrono::steady_clock;

    static constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNan = Eigen::Vector2d{kNan, kNan};

private:
    enum class Status {
        NORMAL,
        ROAD,
        ATTACK,
        STEP,
        SLOPE,
        END,
    };
    Fsm<Status> fsm{Status::NORMAL};

    NodeWrap<rclcpp::Node> wrap;

    LowPassFilter<Eigen::Vector2d> road_low_pass{Tau<1.0>{}, Eigen::Vector2d::Zero()};
    Eigen::Vector2d road_last_position = kVecNan;
    clock::time_point road_last_progress_time = clock::now();
    clock::time_point road_reverse_until = clock::now();

    static auto normalize_yaw(double yaw) -> double {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

    static auto scale_to_min_speed(Eigen::Vector2d speed, double min_speed) -> Eigen::Vector2d {
        constexpr auto kEps = 1e-9;

        const auto norm = speed.norm();
        if (norm < kEps || norm >= min_speed)
            return speed;

        return speed * (min_speed / norm);
    }

    auto current_position() const -> Eigen::Vector2d { return {context.x, context.y}; }

    auto gimbal_toward() const -> Eigen::Vector2d {
        auto target_yaw = normalize_yaw(
            context.target_gimbal_toward.x() - context.current_world_yaw
            + context.current_local_yaw);
        return {target_yaw, context.target_gimbal_toward.y()};
    }

public:
    struct Context {
        Eigen::Vector2d target_chassis_speed = kVecNan; // base_link 系，控制量
        Eigen::Vector2d target_gimbal_toward = kVecNan; // world 系 {yaw, pitch}，控制量
        double current_local_yaw = kNan; // 当前云台在Imu系的观测量（控制高频依赖，但会缓慢漂移）
        double current_world_yaw = kNan; // 当前云台在世界系的观测量
        double x = kNan, y = kNan;

        bool under_attack = false;
    } context;

    struct Command {
        ChassisMode chassis_mode = ChassisMode::ALIGNMENT;
        Eigen::Vector2d chassis_speed = kVecNan; // LidarLink 系
        Eigen::Vector2d gimbal_toward = kVecNan; // OdomLink 系 {yaw, pitch}
    } command;

    explicit MotionFsm(rclcpp::Node& node)
        : wrap{node} {
        fsm.use<Status::NORMAL>(
            [this] { wrap.info("MotionFsm::Enter | NORMAL"); },
            [this] {
                command.chassis_mode = ChassisMode::AUTO;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = gimbal_toward();
                return Status::NORMAL;
            });
        fsm.use<Status::ROAD>(
            [this] {
                wrap.info("MotionFsm::Enter | ROAD");

                const auto now = clock::now();

                road_low_pass.clean(context.target_chassis_speed);
                road_last_position = current_position();
                road_last_progress_time = now;
                road_reverse_until = now;
            },
            [this] {
                constexpr auto kInterval = std::chrono::milliseconds{500};
                constexpr auto kDuration = std::chrono::milliseconds{500};
                constexpr auto kMinSpeed = 0.5;
                constexpr auto kDistanceLimit = 0.1;
                constexpr auto kEps = 1e-9;

                const auto now = clock::now();
                const auto position = current_position();

                command.chassis_mode = ChassisMode::ALIGNMENT;
                command.gimbal_toward = gimbal_toward();

                auto speed = road_low_pass.update(context.target_chassis_speed);
                speed = scale_to_min_speed(speed, kMinSpeed);

                if ((position - road_last_position).norm() >= kDistanceLimit) {
                    road_last_position = position;
                    road_last_progress_time = now;
                }
                if (now >= road_reverse_until) {
                    const auto stuck_for = now - road_last_progress_time;
                    if (stuck_for >= kInterval && speed.norm() >= kEps) {
                        road_last_progress_time = now;
                        road_reverse_until = now + kDuration;
                    }
                }
                command.chassis_speed = (now < road_reverse_until) ? -speed : speed;

                return Status::ROAD;
            });
        fsm.use<Status::ATTACK>(
            [this] { wrap.info("MotionFsm::Enter | ATTACK"); },
            [this] {
                command.chassis_mode =
                    context.under_attack ? ChassisMode::SPIN_FAST : ChassisMode::SPIN_SLOW;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = gimbal_toward();
                return Status::ATTACK;
            });
        fsm.use<Status::STEP>(
            [this] { wrap.info("MotionFsm::Enter | STEP"); },
            [this] {
                command.chassis_mode = ChassisMode::ALIGNMENT;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = gimbal_toward();
                return Status::STEP;
            });
        fsm.use<Status::SLOPE>(
            [this] { wrap.info("MotionFsm::Enter | SLOPE"); },
            [this] {
                command.chassis_mode = ChassisMode::AUTO;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = gimbal_toward();
                return Status::SLOPE;
            });
        if (!fsm.fully_registered())
            throw std::runtime_error{"MotionFsm is not fully registered"};
    }

    auto switch_mode(const std::string& mode) {
        /*^^*/ if (mode == "normal") {
            fsm.start_on(Status::NORMAL);
        } else if (mode == "road") {
            fsm.start_on(Status::ROAD);
        } else if (mode == "attack") {
            fsm.start_on(Status::ATTACK);
        } else if (mode == "step") {
            fsm.start_on(Status::STEP);
        } else if (mode == "slope") {
            fsm.start_on(Status::SLOPE);
        }
    }

    auto spin_once() noexcept {
        {
            command.chassis_mode = ChassisMode::ALIGNMENT;
            command.chassis_speed = kVecNan;
            command.gimbal_toward = kVecNan;
        }
        fsm.spin_once();
        return command;
    }
};

} // namespace rmcs::navigation
