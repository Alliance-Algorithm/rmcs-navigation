#pragma once
#include "util/pimpl.hh"
#include <rclcpp/node.hpp>

#include <tuple>

namespace rmcs::navigation::details {

class Navigation {
    RMCS_PIMPL_DEFINITION(Navigation)
public:
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
    auto send_target(double x, double y, double yaw) -> void;

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

    /// 查询底部云台轴当前朝向（world 坐标系 yaw）。
    ///
    /// 行为说明：
    /// - 若 TF 查询失败，会返回 nan。
    auto check_bottom_yaw() const -> double;

    /// 查询当前活跃导航目标点（world 坐标系）。
    ///
    /// 返回值：
    /// - tuple 第 1 项：goal x（米）；
    /// - tuple 第 2 项：goal y（米）。
    /// - tuple 第 3 项：goal yaw（弧度，若无有效朝向则为 nan）。
    ///
    /// 行为说明：
    /// - 若当前无活跃 action 目标，返回 {nan, nan, nan}。
    auto check_active_goal() const -> std::tuple<double, double, double>;

    /// 取消当前活跃导航目标。
    auto cancel_target() -> void;

    /// 开关 goal topic 转发功能（可选）。
    ///
    /// 行为说明：
    /// - enable=true 时，订阅 `/move_base_simple/goal` 和 `/goal_pose`；
    /// - 收到 PoseStamped 后，会提取 `pose.position.{x,y}` 与 `pose.orientation yaw` 并调用 send_target；
    /// - enable=false 时，取消上述订阅并停止转发。
    ///
    /// 用法示例：
    /// - navigation.switch_topic_forward(true);
    /// - navigation.switch_topic_forward(false);
    auto switch_topic_forward(bool enable) -> void;
};

} // namespace rmcs::navigation::details
