#pragma once
#include "cxx/util/pimpl.hh"
#include <rclcpp/node.hpp>
#include <rmcs_msgs/robot_color.hpp>

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

    /// 取消当前导航目标。
    ///
    /// 行为说明：
    /// - 清除活跃目标并使在途回调失效；
    /// - 若存在已接受的 goal，向 NAV2 发送取消请求；
    /// - 没有活跃目标时为 no-op。
    ///
    /// 用法示例：
    /// - navigation.cancel_target();
    auto cancel_target() -> void;

    /// 触发 rmcs_localization 重定位。
    ///
    /// 行为说明：
    /// - color 为 RED/BLUE 时，向 FIFO
    ///   /tmp/rmcs-localization/relocalize/trigger 写入 "red"/"blue"；
    /// - color 为 UNKNOWN 时直接拒绝并告警，不做兜底；
    /// - FIFO 不可用（定位端未启动）时，本次请求不会发送；
    /// - 重定位结果由定位端写入 /tmp/rmcs-navigation/relocalize/result，
    ///   本端监听并打印成功/失败日志。
    ///
    /// 用法示例：
    /// - navigation.relocalize(rmcs_msgs::RobotColor::RED);
    auto relocalize(rmcs_msgs::RobotColor color) -> void;

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
