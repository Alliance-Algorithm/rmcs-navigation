#include "cxx/controller/motion.hh"
#include "cxx/util/fsm.hh"
#include "cxx/util/math/low_pass.hh"
#include "cxx/util/node_mixin.hh"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rmcs::navigation {

struct MotionFsm::Impl {
    using clock = std::chrono::steady_clock;

    enum class Status {
        NORMAL,
        ROAD,
        ATTACK,
        STEP,
        SLOPE,
        CLIMB,
        SUPPORT_ARM,
        END,
    };
    Fsm<Status> fsm{Status::NORMAL};

    Context& context;
    Command& command;
    NodeWrap<rclcpp::Node> wrap;

    LowPassFilter<Eigen::Vector2d> road_filter{Tau<1.0>{}, Eigen::Vector2d::Zero()};
    Eigen::Vector2d road_last_position = kVecNan;
    clock::time_point road_last_progress_time = clock::now();
    clock::time_point road_reverse_until = clock::now();

    LowPassFilter<Eigen::Vector2d> slope_filter{Tau<3.0>{}, Eigen::Vector2d::Zero()};

    bool yaw_bias_initialized = false;
    double yaw_bias = kNan;
    double last_world_yaw = kNan;

    static constexpr auto normalize_yaw(double yaw) -> double {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

    static auto scale_to_min_speed(Eigen::Vector2d speed, double min) {
        constexpr auto kEps = 1e-9;

        const auto norm = speed.norm();
        if (norm < kEps || norm >= min)
            return speed;

        return Eigen::Vector2d{speed * (min / norm)};
    }

    auto current_position() const { return Eigen::Vector2d{context.x, context.y}; }

    auto update_gimbal_target() -> Eigen::Vector2d {
        constexpr auto kGimbalFree = std::numeric_limits<double>::min();
        if (context.target_gimbal_toward.x() == kGimbalFree
            && context.target_gimbal_toward.y() == kGimbalFree) {
            return {kGimbalFree, kGimbalFree};
        }

        constexpr auto kYawBiasCorrectionGain = 0.002;

        const auto target_yaw = context.target_gimbal_toward.x();
        const auto pitch = context.target_gimbal_toward.y();
        const auto wy = context.current_world_yaw;
        const auto ly = context.current_local_yaw;

        // world 不可用：目标按 OdomGimbalImu 绝对角直通（与 current_local_yaw 同源）
        if (!std::isfinite(wy)) {
            if (!std::isfinite(target_yaw)) {
                return {command.gimbal_toward.x(), command.gimbal_toward.y()};
            }
            return {normalize_yaw(target_yaw), pitch};
        }

        if (std::isfinite(ly)) {
            const auto measured_bias = normalize_yaw(wy - ly);

            if (!yaw_bias_initialized) {
                yaw_bias = measured_bias;
                last_world_yaw = wy;
                yaw_bias_initialized = true;
            } else {
                const auto bias_error = normalize_yaw(measured_bias - yaw_bias);
                yaw_bias = normalize_yaw(yaw_bias + kYawBiasCorrectionGain * bias_error);
                last_world_yaw = wy;
            }
        }

        const auto target_local_yaw = normalize_yaw(target_yaw - yaw_bias);
        return {target_local_yaw, pitch};
    }

    /// 底盘航向误差：与云台共用同一 OdomGimbalImu 目标和测量，不重算 world→odom。
    /// err = gimbal_toward.x − base_odom — 两者随系同漂，差稳定。
    auto update_chassis_direction_error() -> double {
        const auto target_yaw = command.gimbal_toward.x();
        if (!std::isfinite(target_yaw))
            return kNan;

        const auto ly = context.current_chassis_local_yaw;
        if (!std::isfinite(ly))
            return kNan;

        return normalize_yaw(target_yaw - ly);
    }

    auto update_outputs() -> void {
        command.gimbal_toward = update_gimbal_target();
        command.chassis_direction_error = update_chassis_direction_error();
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
                update_outputs();
                return Status::NORMAL;
            });

        /// 过起伏路段模式
        ///   - 底盘对齐有劲模式
        ///   - 速度过 LowPass，缓慢变化
        ///   - 行进卡住则倒退
        fsm.use<Status::ROAD>(
            [this] {
                wrap.info("MotionFsm::Enter | ROAD");

                road_last_progress_time = clock::now();
                road_reverse_until = clock::now();
                road_filter.clean(context.target_chassis_speed);
                road_last_position = current_position();
            },
            [this] {
                constexpr auto kInterval = std::chrono::milliseconds{500};
                constexpr auto kDuration = std::chrono::milliseconds{500};
                constexpr auto kMinSpeed = 0.5;
                constexpr auto kDistanceLimit = 0.1;
                constexpr auto kEps = 1e-9;

                const auto now = clock::now();
                const auto position = current_position();

                command.chassis_mode = ChassisMode::ALIGNMENT_POWERED;
                update_outputs();

                auto speed = road_filter.update(context.target_chassis_speed);
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

        /// 战斗模式
        fsm.use<Status::ATTACK>(
            [this] { wrap.info("MotionFsm::Enter | ATTACK"); },
            [this] {
                command.chassis_mode =
                    context.under_attack ? ChassisMode::SPIN_FAST : ChassisMode::SPIN_SLOW;
                command.chassis_speed = context.target_chassis_speed;
                update_outputs();
                return Status::ATTACK;
            });

        /// 下台阶模式
        fsm.use<Status::STEP>(
            [this] { wrap.info("MotionFsm::Enter | STEP"); },
            [this] {
                command.chassis_mode = ChassisMode::ALIGNMENT_POWERED;
                command.chassis_speed = context.target_chassis_speed;
                update_outputs();
                return Status::STEP;
            });

        /// 下坡模式
        fsm.use<Status::SLOPE>(
            [this] {
                wrap.info("MotionFsm::Enter | SLOPE");
                slope_filter.clean(context.target_chassis_speed);
            },
            [this] {
                command.chassis_mode = ChassisMode::AUTO;
                command.chassis_speed = slope_filter.update(context.target_chassis_speed);
                update_outputs();
                return Status::SLOPE;
            });

        /// 爬升模式
        fsm.use<Status::CLIMB>(
            [this] { wrap.info("MotionFsm::Enter | CLIMB"); },
            [this] {
                command.chassis_mode = ChassisMode::CLIMB;
                command.chassis_speed = Eigen::Vector2d::Zero();
                update_outputs();
                return Status::CLIMB;
            });

        /// 支撑臂模式
        fsm.use<Status::SUPPORT_ARM>(
            [this] { wrap.info("MotionFsm::Enter | SUPPORT_ARM"); },
            [this] {
                command.chassis_mode = ChassisMode::SUPPORT_ARM;
                command.chassis_speed = Eigen::Vector2d::Zero();
                update_outputs();
                return Status::SUPPORT_ARM;
            });

        if (!fsm.fully_registered())
            throw std::runtime_error{"MotionFsm is not fully registered"};
    }

    auto switch_mode(const std::string& mode) -> void {
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
        } else if (mode == "climb") {
            fsm.start_on(Status::CLIMB);
        } else if (mode == "support_arm") {
            fsm.start_on(Status::SUPPORT_ARM);
        }
    }

    auto spin_once() noexcept -> Command {
        fsm.spin_once();
        return command;
    }
};

MotionFsm::MotionFsm(rclcpp::Node& node) noexcept
    : pimpl{std::make_unique<Impl>(*this, node)} {}

MotionFsm::~MotionFsm() noexcept = default;

auto MotionFsm::switch_mode(const std::string& mode) -> void { pimpl->switch_mode(mode); }

auto MotionFsm::spin_once() noexcept -> Command { return pimpl->spin_once(); }

} // namespace rmcs::navigation
