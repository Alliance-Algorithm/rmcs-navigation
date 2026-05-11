#pragma once

#include <Eigen/Geometry>
#include <rmcs_msgs/chassis_mode.hpp>

namespace rmcs::navigation {

/// @brief 控制器抽象基类
///
/// 所有输出均在 OdomImuLink 系下：
/// - 底盘速度：base_link 系经 Rotation2D{current_yaw} 变换到 OdomImu
/// - 云台方向：world 系 desired_yaw 减去 current_yaw 得到 OdomImu 系目标
struct IController {
    static constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    static inline auto kVecNan = Eigen::Vector2d{kNan, kNan};

    virtual ~IController() = default;

    using ChassisMode = rmcs_msgs::ChassisMode;

    struct Context {
        Eigen::Vector2d target_chassis_speed = kVecNan; // base_link 系，控制量
        Eigen::Vector2d target_gimbal_toward = kVecNan; // world 系 {yaw, pitch}，控制量
        double current_local_yaw = kNan; // 当前云台在Imu系的观测量（控制高频依赖，但会缓慢漂移）
        double current_world_yaw = kNan; // 当前云台在世界系的观测量
    };

    struct Command {
        Eigen::Vector2d chassis_speed = kVecNan; // OdomImu 系
        ChassisMode chassis_mode = ChassisMode::AUTO;
        Eigen::Vector2d gimbal_toward = kVecNan; // OdomImu 系 {yaw, pitch}
    };

    /// @brief 更新上下文（控制量 + 观测量）
    virtual auto update_context(Context context) -> void = 0;

    /// @brief 生成控制命令（全部在 OdomImuLink 系下）
    virtual auto generate_command() const -> Command = 0;
};

} // namespace rmcs::navigation
