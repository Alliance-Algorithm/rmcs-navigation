#include "cxx/controller/motion.hh"
#include "cxx/util/fsm.hh"
#include "cxx/util/math/low_pass.hh"
#include "cxx/util/node_mixin.hh"

#include <cmath>
#include <stdexcept>

namespace rmcs::navigation {

struct MotionFsm::Impl {
    enum class Status {
        NORMAL,
        ATTACK,
        SLOPE,
        END,
    };
    Fsm<Status> fsm{Status::NORMAL};

    Context& context;
    Command& command;
    NodeWrap<rclcpp::Node> wrap;

    LowPassFilter<Eigen::Vector2d> slope_filter{Tau<3.0>{}, Eigen::Vector2d::Zero()};

    bool gimbal_yaw_bias_initialized = false;
    double gimbal_yaw_bias = kNan;
    std::uint64_t last_yaw_sample = 0;

    static auto normalize_yaw(double yaw) -> double {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

    auto update_yaw_bias(double local_yaw, bool& initialized, double& bias) -> bool {
        const auto world_yaw = context.current_world_yaw;
        if (!std::isfinite(world_yaw) || !std::isfinite(local_yaw))
            return false;

        constexpr auto kYawBiasCorrectionGain = 0.002;
        const auto measured_bias = normalize_yaw(world_yaw - local_yaw);
        if (!initialized) {
            bias = measured_bias;
            initialized = true;
        } else {
            const auto bias_error = normalize_yaw(measured_bias - bias);
            bias = normalize_yaw(bias + kYawBiasCorrectionGain * bias_error);
        }
        return true;
    }

    auto update_yaw_biases() -> void {
        if (last_yaw_sample == context.yaw_sample)
            return;

        last_yaw_sample = context.yaw_sample;
        update_yaw_bias(
            context.current_gimbal_yaw, gimbal_yaw_bias_initialized, gimbal_yaw_bias);
    }

    auto world2gimbal_odom(double world_yaw) const -> double {
        if (!std::isfinite(world_yaw) || !gimbal_yaw_bias_initialized)
            return kNan;
        return normalize_yaw(world_yaw - gimbal_yaw_bias);
    }

    auto update_gimbal_target() -> Eigen::Vector2d {
        constexpr auto kGimbalFree = std::numeric_limits<double>::min();
        if (context.target_gimbal_toward.x() == kGimbalFree
            && context.target_gimbal_toward.y() == kGimbalFree) {
            return {kGimbalFree, kGimbalFree};
        }

        const auto target_yaw = context.target_gimbal_toward.x();
        const auto pitch = context.target_gimbal_toward.y();
        const auto wy = context.current_world_yaw;

        // world 不可用时保持上一帧云台目标，避免切换坐标参考。
        if (!std::isfinite(wy)) {
            return {command.gimbal_toward.x(), command.gimbal_toward.y()};
        }

        const auto target_local_yaw = world2gimbal_odom(target_yaw);
        return {target_local_yaw, pitch};
    }

    Impl(MotionFsm& owner, rclcpp::Node& node)
        : context{owner.context}
        , command{owner.command}
        , wrap{node} {

        /// 普通模式
        fsm.use<Status::NORMAL>(
            [this] { wrap.info("MotionFsm::Enter | NORMAL"); },
            [this] {
                command.chassis_mode = ChassisMode::ALIGNMENT;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = update_gimbal_target();
                return Status::NORMAL;
            });

        /// 战斗模式
        fsm.use<Status::ATTACK>(
            [this] { wrap.info("MotionFsm::Enter | ATTACK"); },
            [this] {
                command.chassis_mode =
                    context.under_attack ? ChassisMode::SPIN_FAST : ChassisMode::SPIN_SLOW;
                command.chassis_speed = context.target_chassis_speed;
                command.gimbal_toward = update_gimbal_target();
                return Status::ATTACK;
            });

        /// 下坡模式
        fsm.use<Status::SLOPE>(
            [this] {
                wrap.info("MotionFsm::Enter | SLOPE");
                slope_filter.clean(context.target_chassis_speed);
            },
            [this] {
                command.chassis_mode = ChassisMode::ALIGNMENT;
                command.chassis_speed = slope_filter.update(context.target_chassis_speed);
                command.gimbal_toward = update_gimbal_target();
                return Status::SLOPE;
            });

        if (!fsm.fully_registered())
            throw std::runtime_error{"MotionFsm is not fully registered"};
    }

    auto switch_mode(const std::string& mode) -> void {
        /*^^*/ if (mode == "normal") {
            fsm.start_on(Status::NORMAL);
        } else if (mode == "attack") {
            fsm.start_on(Status::ATTACK);
        } else if (mode == "slope") {
            fsm.start_on(Status::SLOPE);
        }
    }

    auto spin_once() noexcept -> Command {
        update_yaw_biases();
        fsm.spin_once();
        return command;
    }
};

MotionFsm::MotionFsm(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(*this, node)} {}

MotionFsm::~MotionFsm() noexcept = default;

auto MotionFsm::switch_mode(const std::string& mode) -> void { pimpl->switch_mode(mode); }

auto MotionFsm::world2gimbal_odom(double yaw) -> double { return pimpl->world2gimbal_odom(yaw); }

auto MotionFsm::spin_once() noexcept -> Command { return pimpl->spin_once(); }

} // namespace rmcs::navigation
