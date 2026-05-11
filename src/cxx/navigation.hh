#pragma once
#include "cxx/util/pimpl.hh"
#include <rclcpp/node.hpp>

#include <Eigen/Geometry>

namespace rmcs::navigation::details {

class Navigation {
    RMCS_PIMPL_DEFINITION(Navigation)
public:
    struct Command {
        Eigen::Vector2d speed;
        std::chrono::steady_clock::time_point timestamp;
    };

    explicit Navigation(rclcpp::Node& node) noexcept;

    /// 向 NAV2 下发 2D 导航目标（world 坐标系）。
    ///
    /// 行为说明：
    /// - 若与当前活跃目标在 epsilon 内相同，则直接忽略；
    /// - 若不同，会取消上一条活跃目标并发送新目标；
    /// - 若 action server 不可用，本次请求不会发送。
    ///
    /// 用法示例：
    /// - navigation.send_target(1.5, -2.0);
    auto send_target(double x, double y) -> void;

    /// 查询当前位姿（world -> base_link）。
    ///
    /// 返回值：
    /// - tuple 第 1 项：x（米）；
    /// - tuple 第 2 项：y（米）；
    /// - tuple 第 3 项：yaw（弧度）。
    ///
    /// 行为说明：
    /// - 若 TF 查询失败，会返回 {nan, nan, nan}。
    ///
    /// 用法示例：
    /// - auto [x, y, yaw] = navigation.check_position();
    auto check_position() const -> std::tuple<double, double, double>;

    /// 开关 goal topic 转发功能（可选）。
    ///
    /// 行为说明：
    /// - enable=true 时，订阅 `/move_base_simple/goal` 和 `/goal_pose`；
    /// - 收到 PoseStamped 后，会提取 `pose.position.{x,y}` 并调用 send_target；
    /// - enable=false 时，取消上述订阅并停止转发。
    ///
    /// 用法示例：
    /// - navigation.switch_topic_forward(true);
    /// - navigation.switch_topic_forward(false);
    auto switch_topic_forward(bool enable) -> void;

    /// 获取最近一次 cmd_vel 速度和时间戳。
    auto current_command() const -> Command;
};

} // namespace rmcs::navigation::details
