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

录制模式（`record.lua`）下拨杆切回关闭录制后，会自动启动 `scripts/record-map.sh` 完成离线建图，不需要再逐条执行下面的手动步骤：

1. 小电脑端：停止录包 → 校验 bag → 停止实时导航 → 回放 bag → 启动 `point_lio` → 保存 PCD → 复制到 `~/record-livox/maps/<bag 名>.pcd` → 重启导航。
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
tail -f /tmp/rmcs-navigation/record-map.log
tail -f /tmp/rmcs-navigation/point-lio.log
```

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