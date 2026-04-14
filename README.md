# RMCS NAVIGATION

## 0. 基本架构

`RMCS-NAVIGATION` 是一个 RoboMaster 哨兵机器人自主导航决策系统，采用 C++ 和 Lua 开发主要逻辑，该程序以 [RMCS 控制系统插件](https://github.com/Alliance-Algorithm/RMCS) 的形式运行，由 RMCS 提供运行时与上下文，由 ROS Navigation 提供导航能力，Lua 开发决策

- `Component`：获取机器人状态，裁判系统等信息，提供运动控制接口，以及为 Lua 侧提供运行时
- `ROS Navigation`：Nav2 堆栈的纯配置与启动文件，负责路径规划及其他导航相关能力
- `Lua Decision`：在 `component` 的 update 中自旋，其热重载特性和原生协程支持有利于决策的快速迭代开发

## 1. 快速入门

### 信息流与调用链总览

![Call Chain](doc/call-chain.svg)

### Component

配置项：

```yaml
rmcs_navigation:
  ros__parameters:
    command_vel_name: "/cmd_vel_smoothed"
    mock_context: true
    endpoint: "main"
    decision: "fast-push-output"
    # ......
```
