# AGENTS.md

## Mission

- 本 skill 用于指导 RMCS 导航相关开发与排障（ROS2 工作区位于 `rmcs_ws`），目标是在**最小改动**前提下快速定位问题并稳定交付可运行结果。
- 面向导航链路（如定位、建图、传感器驱动与进程协同）提供可复现的调试步骤，避免残留进程、环境漂移和“本地可用/远端不可用”的不一致。
- 输出应包含可直接执行的命令与验证标准，默认优先保证系统安全性、正确性和可维护性。

## Global Rules

### 1) 项目通用构建方法

- 对于处在 `rmcs_ws` 下的标准 ROS pkg：

```zsh
# 在 zsh 中，所有环境均已配置完好
build-rmcs

# 如果只编译单个包
build-rmcs --packages-select xxxx
build-rmcs --packages-up-to xxxx

# 清空编译产物
clean-rmcs
```

- 对于 `tool/`、`test/` 这类项目内独立工程：

```zsh
cmake -B build
cmake --build build -j
```

### 2) C++ 语言风格

- 构造和声明变量

```
# 使用大括号构造和类型后置
auto var = T { };
```

## Debugging SOP

### 1) 导航相关调试方法

- 使用 `screen` 管理进程（例如 `point-lio`、`livox-ros-driver`）；这些进程容易遗留，或因死锁变为僵尸进程。

### 2) 雷达适配 SOP（topic + 变换）

- 先确认系统实际发布的雷达话题（常见如 `/livox/lidar_192_168_100_120` 与 `/livox/imu_192_168_100_120`），不要假设是 `/livox/lidar`。

#### A. 话题适配（必须一致）

- `rmcs_local_map`：修改 `rmcs_ws/src/skills/rmcs_local_map/config/local_map.yaml`
  - `lidar.lid_topic`
  - `lidar.imu_topic`
- `point_lio`：修改 `rmcs_ws/src/skills/point_lio/config/mid360.yaml`
  - `common.lid_topic`
  - `common.imu_topic`
- 原则：`rmcs_local_map` 与 `point_lio` 必须订阅同一套雷达 topic（同一 IP 后缀）。

#### B. 雷达到底盘装配位姿适配

- `rmcs_local_map`：写入 `rmcs_ws/src/skills/rmcs_local_map/config/local_map.yaml`
  - `lidar.lidar_translation: [x, y, z]`
  - `lidar.lidar_orientation: [yaw, pitch, roll]`（单位：度）
- `point_lio`：写入 `rmcs_ws/src/skills/point_lio/config/mid360.yaml`
  - `common.init_pose.translation: [x, y, z]`
  - `common.init_pose.orientation: [yaw, pitch, roll]`（单位：度，表示 `base_link -> lidar_link`）
- 原则：`rmcs_local_map` 与 `point_lio` 的雷达装配位姿必须来自同一套标定数据。

#### C. 验证步骤

- 构建（zsh）：

```zsh
build-rmcs --packages-select point_lio rmcs_local_map rmcs-navigation
```

- 启动后检查：
  - `ros2 topic list | rg 'livox|local_map|map'` 确认订阅/发布链路存在。
  - `ros2 param get /laserMapping common.init_pose.translation` 与 `common.init_pose.orientation` 确认参数加载正确。
  - `ros2 run tf2_ros tf2_echo world camera_init` 检查安装位姿是否为期望值。
  - `ros2 run tf2_ros tf2_echo aft_mapped base_link` 检查是否仅有 `Z` 平移。
  - 对齐检查以同一 frame 进行（优先 `base_link`），避免跨 frame 误判为“轴错位”。

#### E. 常见故障

- 非 ASCII 路径 YAML 读取失败：若 launch 读取 `custom.yaml`，使用 `encoding="utf-8"`，避免 `UnicodeDecodeError`。
- TF 断树：若 `world` 与 `base_link` 不连通，先检查 `point_lio` 是否正常发布 `camera_init <-> aft_mapped` 及静态 TF 是否起效。
