#pragma once
#include "common.hh"

namespace rmcs::navigation {

/// @brief 正常模式：将 base_link 系速度变换到 OdomImu 系，将 world 系目标方向变换到 OdomImu 系。
/// 用于巡逻、正常位移、战斗。
struct NormalController : IController {
    auto update_context(Context ctx) -> void override { context = ctx; }

    auto generate_command() const -> Command override {
        // 底盘：base_link → OdomImu
        auto rotated = Eigen::Rotation2Dd{context.current_local_yaw} * context.target_chassis_speed;

        // 云台：world → OdomImu
        auto target_yaw = normalize_yaw(
            context.target_gimbal_toward.x() - context.current_world_yaw
            + context.current_local_yaw);
        return {
            .chassis_speed = rotated,
            .chassis_mode = ChassisMode::AUTO,
            .gimbal_toward = {target_yaw, context.target_gimbal_toward.y()},
        };
    }

private:
    static auto normalize_yaw(double yaw) -> double {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

    Context context;
};

} // namespace rmcs::navigation
