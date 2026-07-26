# RMCS NAVIGATION

## 0. 基本架构

`RMCS-NAVIGATION` 是一个 RoboMaster 哨兵机器人自主导航决策系统，采用 C++ 和 Lua 开发主要逻辑，该程序以 [RMCS 控制系统插件](https://github.com/Alliance-Algorithm/RMCS) 的形式运行，由 RMCS 提供运行时与上下文，由 ROS Navigation 提供导航能力，Lua 开发决策

- `Component`：获取机器人状态，裁判系统等信息，提供运动控制接口，以及为 Lua 侧提供运行时
- `ROS Navigation`：Nav2 堆栈的纯配置与启动文件，负责路径规划及其他导航相关能力
- `Lua Decision`：在 `component` 的 update 中自旋，其热重载特性和原生协程支持有利于决策的快速迭代开发

## 1. 快速入门

### 信息流与调用链总览

![Call Chain](doc/call-chain.svg)

### Component (C++ 侧)

一个最小的可运行示例如下：

```yaml
# rmcs_bringup/config/navigation.yaml
rmcs_executor:
  ros__parameters:
    update_rate: 1000.0
    components:
      - rmcs::navigation::Navigation -> rmcs_navigation

rmcs_navigation:
  ros__parameters:
    # 策略名称：
    # - fast-push-output "速推前哨站"
    # - kill-robots "杀伤优先"
    decision: "fast-push-output"
    command_vel_name: "/cmd_vel_smoothed"
    endpoint: "main"
    enable_goal_topic_forward: true
```

构建使用如下指令在本机启动：
```zsh
ros2 launch rmcs_bringup rmcs.launch.py robot:=navigation
```

本地注入裁判上下文（FIFO，无需 ROS topic）：
```zsh
local-context game_stage started
local-context robot_health 350
local-context robot_bullet 200
# → /tmp/rmcs-navigation/context/{game_stage,robot_health,robot_bullet}
```

### Decision (Lua 侧)

#### 1. 基本思想

将任务拆解成可复现，可单测的最小示例，例如，对于开局后前往巡逻点的任务，我们将其拆分为

```
核心任务：进行巡航打击

0. 从起始点前往公路区前（起伏路段）
{
  a. 将导航目标点设置到公路区前
  b. 阻塞检查坐标，直到到达目的地
}

1. 跨越起伏路段（begin -> final）
{
  a. 设置底盘跟随
  b. 设置云台与起伏路段垂直
  c. 切换底盘至起伏路段模式
  d. 开始导航至起伏路段对面
}

2. 进入巡航模式
{
  a. 开启小陀螺
  b. 设置云台为扫描模式
  c. 循环
    - 到达点1
    - 看看狗洞
    - 到达点2
    - 看看前哨站
    - 下一次循环
}
```

我们或许有这样的文件结构：
```
lua/
  intent/                       意图，核心任务，是下面迷你任务的组合
    start-cruise.lua            > 开始巡航

  task/                         迷你任务
    navigate-to.lua             > 导航至（普通事件，不涉及地形跨越）
    cruise-loop.lua             > 巡航进行时

  endpoint/                     接入点
    main.lua                    > 比赛用的接入点，是正式入口
    test.lua                    > 测试用的，随便改动
```

对于线下调试，我们可以直接把小任务（比如从起始点前往公路区前）注册进调度器，再绑定一个遥控器触发信号为开始，手动触发该事件，单独测试

```lua

on_init = function()
  ...
  -- 右拨杆向上的触发事件
  edges:on(blackboard.getter.rswitch, "UP", function()
    --- 触发一个小任务
    scheduler:append_task(function()
      -- 小任务可以是阻塞的，使用协程可以很轻松处理多个
      -- 语义上阻塞任务的并行，使用线性语义描述业务是美妙的
      crossing_road_zone {
        begin = {x = 0, y = 0},
        final = {x = 3, y = 0},
      }

      -- 再回来
      crossing_road_zone {
        begin = {x = 3, y = 0},
        final = {x = 0, y = 0},
      }
	  end)
	end)
  ...
end

```


#### 2. 框架使用

> 该框架的核心目标，是将高层决策与底层执行明确分离：最上层由 `endpoint` 根据当前局势持续观察和判断，在运行中决定机器人此刻应当采取什么意图，例如继续推进、回防基地或撤退补血；意图一旦确定，再被拆解为可顺序执行、可组合的任务流程；这些任务不直接调用底层能力，通过 `action` 这一层发起具体动作，由它封装运行时状态并收口到底层 `api`；与此同时，整个过程依赖 `blackboard` 提供共享的世界状态与上下文信息。

对于接入点（endpoint），需要暴露以下接口：

```lua
-- 状态黑板
blackboard

-- Lua 侧的初始化
on_init

-- 10Hz 的更新 Tick
on_tick

-- 退出时的 Hook
on_exit (optional)

-- 收到来自 nav2 的控制 Topic 的回调
on_control
```

TODO: 未完待续
