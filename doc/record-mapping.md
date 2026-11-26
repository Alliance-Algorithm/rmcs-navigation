# 雷达录包建图 SOP

> 本文档整理"雷达录包 → 小电脑离线建图 → PCD 地图回传主机"的完整流程，来自实机验证过的操作过程。
>
> 适用场景：
> - 使用 Livox MID360 录包，在机器人（小电脑）上离线回放建图
> - 产出 `point_lio` 的全局 PCD 地图，供重定位（`rmcs-localization`）与 Nav2 全局地图使用

---

## 1. 流程总览

```
主机(录包?)/小电脑录包
    ↓  ssh 连接小电脑
小电脑: 清理残留会话 → 回放 bag → 启动 point_lio 建图 → 触发保存 PCD
    ↓  HTTP / scp
主机: curl 下载 PCD 地图
```

建图核心是 `point_lio`（节点名 `laserMapping`）：订阅雷达+IMU 话题跑 SLAM，内部持续累积全局点云，收到 `save_pcd_map` 服务调用后把累积点云保存为 `.pcd` 文件。

## 2. 自动化流程（推荐）

录制功能以普通调度任务提供，普通 endpoint 默认不启用。在需要此功能的 endpoint 中引入，并在 `on_init` 中注册到现有调度器：

```lua
local auto_record = require("task.auto_record")

-- 在现有 on_init 中加入：
scheduler:append_task(auto_record)
```

各 endpoint 按需显式注册此任务。接入后不需要再切换 `endpoint` 配置来录制；仅在没有比赛、训练任务运行的待机状态下使用：

1. 在 **2 秒内连续产生三次旋钮 `MIDDLE → DOWN` 下降沿**，即 `中→下→中→下→中→下`。只计下降沿，保持在下不重复计数，向上或遥控器断连会清除未完成的手势。
2. 录包进程实际启动后，调用 `action:gimbal_nod(2)` 点头两次作为开始信号。此后左右拨杆可按正常遥控方式使用，改变档位不会结束录制。
3. 再重复同一组三次下降沿，结束录制，调用 `action:gimbal_nod(2)` 点头两次，并自动离线建图。开始、结束信号会排队执行，不会互相覆盖。点头期间需要遥控器处于允许云台运动的状态。
4. 建图期间忽略录制手势；完成或失败后关闭导航、传感器及离线建图进程，原始 bag 和最终 PCD 保留。

`auto_record` 不切换、保存或回退底盘运动模式，也不保存或恢复导航启动配置。结束后关闭导航控制和目标转发、挂起云台导航；正常任务结束时，完成进程清理后才写入最终状态并释放调度器。成功、失败和取消均保持导航关闭；正式导航前由 endpoint 显式调用 `action:restart_navigation { ... }` 重新启动，并确认定位就绪。

`auto_record` 是协程任务，不修改 endpoint 的 `on_init`、`on_tick`、`on_exit`。现有 `on_tick` 继续更新时间并执行 `scheduler:spin_once()` 即可。仅 `require("task.auto_record")` 不会启用录制，注册后才开始监听手势。

录制、建图期间，任务临时独占同一个调度器，暂停其他业务任务，保留遥控移动，并通过自己的云台任务执行两次点头；完成或失败后释放调度器。任务注册顺序不影响接管后的独占行为。取消该任务会关闭导航控制；正在录包时还会请求关闭录包、清理导航和传感器并保留 bag。

独立 `record-workflow` tmux 任务负责录包及建图，雷达尚未启动时会临时启动 Livox 驱动。

结束录制使用 `SIGTERM` 请求 `ros2 bag record` 正常停止并写盘，等待进程以退出码 0 结束后再进入建图。非交互 shell 的后台录包可能继承忽略 `SIGINT`，不能依赖该信号停止。写盘最多等待约 30 秒；超时强制结束或异常退出均报告失败、保留原始包，并跳过建图。

录制进程管理使用独立 C++ 工作线程：启动 shell、加载环境、tmux 探活、停止请求和状态文件读写均在后台执行。Lua 实时更新仅提交请求并读取缓存，不等待外部命令。请求接受后进入 `starting`，实际录包就绪后才点头两次；启动失败会通过状态上报。首次使用会异步检查是否存在上次 rmcs 服务退出后仍运行的录制任务，检查完成前不接受手势。

注意：旧哨兵的 `SentryClimber` 也会处理旋钮 DOWN 落地请求；该底层行为仍存在。折叠哨兵的旋钮向上折叠手势不受本次向下计数影响。

自动化流程：

1. 小电脑端：停止录包 → 校验雷达与 IMU 均有消息 → 停止实时导航 → 启动独立 `point_lio` → 回放 bag → 保存 PCD → 复制到 `record-livox/maps/<bag 名>.pcd` → 关闭相关导航、传感器和建图进程。
2. 主机端：`map-watch` 轮询小电脑，把新 PCD 自动拉到 `/workspaces/RMCS/`。

主机启动后台监听（只需启动一次，会持续轮询）：

```zsh
map-watch --daemon   # 日志：/tmp/rmcs-map-watch.log
map-watch --stop     # 停止监听
```

只拉取一次：

```zsh
map-watch --once
```

小电脑端的建图状态与日志：

```zsh
cat /tmp/rmcs-navigation/record-map.status
cat /tmp/rmcs-navigation/record.status
tail -f /tmp/rmcs-navigation/record-session.log
tail -f /tmp/rmcs-navigation/record-map.log
tail -f /tmp/rmcs-navigation/point-lio.log
```

`record.status` 显示整个任务状态：`starting`、`recording <bag>`、`mapping <bag>`、`success <pcd>`、`aborted` 或 `failed: <原因>`。后台任务可通过 `tmux attach -t record-workflow` 查看。

录制、回放和保存均在小电脑本地执行，不依赖开发电脑在线。原始 bag 与最终 PCD 保存在 `/home/ubuntu/record-livox/`（无该用户目录时用 `/root/record-livox/`），正常重启不会主动删除；`/tmp` 下仅保存临时输出、状态和日志。建图失败保留 bag，PCD 复制失败不会报告成功。网络恢复后可运行 `map-watch --once` 补传。

如果 bag 记录在 `/root/record-livox` 而不是 `/home/ubuntu/record-livox`，给 `map-watch` 传入
`--remote-dir /root/record-livox/maps`。

注意：`point_lio` 内部点云采集上限为 20 分钟（`point-lio/src/collection.h` 的
`set_time_limit(20min)`），超过 20 分钟的 bag 只会保存前 20 分钟的点云。

下面的手动步骤保留用于调试，或需要跳过自动流程单独重跑建图时参考。

## 3. 完整操作步骤

### 步骤 0：录包（实机）

使用 record 模式录制雷达与 IMU 话题（话题名按实际雷达 IP 后缀，参见 `point_lio/config/mid360.yaml` 的 `common.lid_topic` / `imu_topic`）：

```zsh
或 ros2 bag record /livox/lidar_<ip> /livox/imu_<ip> -o ~/record-livox/livox-<日期>-<时间>
```

录完后确认包有效（**消息数必须 > 0**，否则是空包）：

```zsh
ros2 bag info ~/record-livox/livox-<日期>-<时间>
```

### 步骤 1：SSH 连接小电脑

```zsh
ssh-remote
```

### 步骤 2：查看已有包

```zsh
ls ~/record-livox/
```

### 步骤 3：清理残留会话（避免进程冲突）

```zsh
tmux kill-session -t navigation 2>/dev/null
tmux kill-session -t record-livox 2>/dev/null
```

### 步骤 4：回放 bag（终端 1，保持前台运行）

```zsh
ros2 bag play ~/record-livox/livox-<日期>-<时间>
```

### 步骤 5：启动 point_lio 建图（终端 2，tmux 分窗）

```zsh
ros2 launch point_lio point_lio.launch.py
```

建图节点持续订阅 bag 数据并累积全局地图，等待回放结束。

### 步骤 6：触发保存 PCD

bag 回放结束后（建图节点仍在运行），在终端 2 触发保存：

```zsh
ros2 service call save_pcd_map std_srvs/srv/Trigger "{}"
```

成功后日志输出 `Map has been saved to: <saving_path>/<时间戳>.pcd`。

### 步骤 7：确认 PCD 文件

```zsh
ls /tmp/point-lio/
```

输出示例：`20260828_131734_147.pcd`（文件名格式 `<YYYYMMDD_HHMMSS_fff>.pcd`）。

### 步骤 8：HTTP 传输到主机

小电脑端（在含 PCD 的目录起 HTTP 服务，绑定 0.0.0.0）：

```zsh
cd /tmp/point-lio && python3 -m http.server 8080
```

主机端（小电脑 IP 以实际为准，本例 `192.168.3.27`）：

```zsh
# 先列出可下载的文件，确认文件名
curl -s http://192.168.3.27:8080/ | grep -o '[0-9_]*\.pcd'

# 下载到主机
curl -o /workspaces/RMCS/20260829_142935_693.pcd \ http://192.168.3.27:8080/20260829_142935_693.pcd
```

传输完成后在小电脑 `Ctrl+C` 关闭 HTTP 服务。

### 备选：scp 直传（更简单，跳过 HTTP 服务）

```zsh
scp -P 2022 root@192.168.3.27:/tmp/point-lio/\*.pcd /workspaces/RMCS/
```
