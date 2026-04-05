# rmcs-navigation (Ai Generated, 不保真)

`rmcs-navigation` 是一个由 `rmcs_executor` 加载的导航决策组件，和一个 Nav2 启动包。

它的核心结构是：

- C++ 组件 `rmcs::navigation::Navigation`
- Lua 决策入口 `main.lua`
- Lua 黑板 `blackboard.lua`
- Lua 原生接口表 `api.lua`

组件负责把裁判系统/遥控器上下文同步进 Lua，并执行每帧决策逻辑；Lua 负责行为组织、切换条件与导航相关业务策略。

## 目录结构

- `src/cxx/component.cc`：`rmcs_executor` 组件主实现。
- `src/cxx/util/logger_mixin.hh`：组件日志封装。
- `src/lua/main.lua`：Lua 决策入口，要求定义 `on_init()` 和 `on_tick()`。
- `src/lua/blackboard.lua`：Lua 黑板单例与默认字段。
- `src/lua/api.lua`：Lua 侧可调用的原生接口表。
- `config/rmul.yaml`、`config/rmuc.yaml`：决策配置。
- `launch/*.launch.py`：Nav2/在线导航/自定义模式等启动入口。

## 组件运行模型

`Navigation` 组件在构造阶段完成以下工作：

- 读取参数，例如 `command_vel_name`、`mock_context`
- 初始化上下文输入接口
- 初始化输出接口
- 创建 `Twist` 订阅
- 初始化 Lua VM，并加载 `main.lua` / `blackboard.lua` / `api.lua`
- 调用 Lua `on_init()`

运行阶段每次 `update()` 会执行：

- 将当前上下文同步到 Lua blackboard
- 调用 Lua `on_tick()`

当前实现采用 fail-fast 策略：

- Lua 初始化失败会在构造阶段直接抛异常
- Lua 运行期异常会在 `update()` 中直接抛异常
- 不再使用静默熔断或失败后跳过更新的策略

## 输入输出接口

### 输入

组件当前声明的输入包括：

- `/referee/id`
- `/remote/switch/right`
- `/remote/switch/left`
- `/referee/game/stage`
- `/referee/current_hp`
- `/referee/shooter/bullet_allowance`
- `/referee/game/red_score`
- `/referee/game/blue_score`

另外还会额外订阅速度输入：

- `command_vel_name` 参数指定的话题，默认常见值为 `/cmd_vel_smoothed`

### 输出

组件当前导出的输出接口包括：

- `/rmcs_navigation/chassis_velocity`
- `/rmcs_navigation/nod_count`
- `/rmcs_navigation/rotate_chassis`
- `/rmcs_navigation/detect_targets`
- `/rmcs_navigation/start_autoaim`

其中：

- `/rmcs_navigation/chassis_velocity` 由外部 `Twist` 订阅直接写入
- 其他输出通常由 Lua 逻辑间接控制

## Lua 决策接口

### 必要入口

`src/lua/main.lua` 必须定义：

- `on_init()`
- `on_tick()`

组件初始化时会调用 `on_init()`，之后每帧调用 `on_tick()`。

### Blackboard 结构

Lua 黑板当前默认包含以下字段：

- `user.health`
- `user.bullet`
- `game.stage`
- `play.rswitch`
- `play.lswitch`
- `rule.decision`
- `rule.health_limit`
- `rule.health_ready`
- `rule.bullet_limit`
- `rule.bullet_ready`
- `rule.home`
- `meta.timestamp`

其中由 C++ 每帧同步的主要字段为：

- `user.health`
- `user.bullet`
- `game.stage`
- `play.rswitch`
- `play.lswitch`
- `meta.timestamp`

### Api 注入

`api.lua` 中的 `api` 表会在 C++ 侧被进一步注入原生函数。

当前额外注入了：

- `api.info(message: string)`
- `api.warn(message: string)`

这两个函数由 C++ 直接挂到 `api` 表上，可在 Lua 中直接使用：

```lua
local api = require("api")

api.info("navigation init")
api.warn("health is low")
```

## Mock Context

当参数 `mock_context: true` 时，组件不会从真实输出链路读取上下文，而是将输入接口本地 direct bind，并额外监听一个 mock topic。

### Mock Topic

- topic: `/rmcs_navigation/context/mock`
- type: `std_msgs/msg/String`

### Payload 形式

负载内容是裸 YAML 文本，要求根节点是 map。例如：

```yaml
game_stage: 4
robot_health: 350
```

当前 mock 同步逻辑允许只更新部分字段，不要求一次性提供完整上下文。

### 当前支持的 mock 字段

当前 `Context::from()` 实际支持：

- `game_stage`
- `robot_health`
- `robot_bullet`
- `red_score`
- `blue_score`

当前这几项按数值解析：

- `game_stage` 接受底层整数值
- 其余字段接受整数

注意：虽然组件内部也声明了 `robot_id`、`switch_right`、`switch_left` 输入，但当前 mock YAML 同步逻辑没有覆盖这些字段。

## 远程脚手架

项目根目录提供了脚手架：

- `/workspaces/RMCS/.script/remote-context`

它会向远端发布 `std_msgs/msg/String` 到：

- `/rmcs_navigation/context/mock`

常用示例：

```bash
remote-context game_stage started
remote-context game_stage 4
remote-context robot_health 350
remote-context red_score 3
```

其中脚手架会把：

- `started` 转成 `4`
- `preparation` 转成 `1`
- `unknown` 转成 `255`

因此可以直接用较友好的文本形式发 `game_stage`。

## 参数

常用参数包括：

- `command_vel_name`：速度输入订阅话题
- `mock_context`：是否启用本地 mock context 模式

一个最小示例见：

- `rmcs_bringup/config/navigation.yaml`

当前安装配置示例为：

```yaml
rmcs_navigation:
  ros__parameters:
    command_vel_name: "/cmd_vel_smoothed"
    mock_context: true
```

## 构建

在 RMCS 开发环境（zsh）中：

```bash
build-rmcs --packages-select rmcs-navigation
```

如果还要联动构建定位与地图链路：

```bash
build-rmcs --packages-select point_lio rmcs_local_map rmcs-navigation
```

## 调试

### 1) 组件初始化失败

优先检查：

- `main.lua` 是否正确定义 `on_init()` / `on_tick()`
- `api.lua`、`blackboard.lua` 是否能被 `require(...)` 到
- 包内 share 目录中的 Lua 文件是否已正确安装

### 2) mock 模式不生效

优先检查：

- `mock_context` 是否为 `true`
- 是否向 `/rmcs_navigation/context/mock` 发了消息
- 消息负载是否是根节点为 map 的 YAML 文本

### 3) Lua 侧验证

可以在 Lua 中直接调用：

```lua
local api = require("api")
api.info("lua tick reached")
```

用于确认 `api` 注入与决策执行链路是否正常。

## Launch 相关

本包仍然保留 Nav2 与在线导航相关 launch 文件，例如：

- `launch/nav2.launch.py`
- `launch/online.launch.py`
- `launch/custom.launch.py`
- `launch/follow_waypoints.launch.py`
