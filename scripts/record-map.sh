#!/usr/bin/env bash

set -o pipefail

state_dir=/tmp/rmcs-navigation
bag_state="$state_dir/last-record-bag"
status_file="$state_dir/record-map.status"
log_file="$state_dir/record-map.log"
point_lio_log="$state_dir/point-lio.log"
marker=""

lio_pid=""
play_pid=""
stack_stopped=0

mkdir -p "$state_dir"
exec >>"$log_file" 2>&1

log() {
    printf '[record-map %s] %s\n' "$(date '+%F %T')" "$*"
}

set_status() {
    printf '%s\n' "$*" >"$status_file"
}

if [ -f "$HOME/env_setup.bash" ]; then
    env_file="$HOME/env_setup.bash"
elif [ -f /root/env_setup.bash ]; then
    env_file=/root/env_setup.bash
else
    log "env_setup.bash not found"
    set_status "failed: env_setup.bash not found"
    exit 1
fi

source "$env_file"
export RCUTILS_COLORIZED_OUTPUT=0

restart_navigation() {
    local configs="launch_livox:=true launch_odin1:=false global_map:=empty use_sim_time:=false"
    log "restarting navigation stack"
    tmux kill-session -t navigation 2>/dev/null || true
    tmux new-session -d -s navigation -n motion \
        "bash -lc 'source \"$env_file\" && ros2 launch rmcs-navigation motion.launch.yaml $configs'"
    tmux new-window -t navigation -n sensor \
        "bash -lc 'source \"$env_file\" && ros2 launch rmcs-navigation sensor.launch.yaml $configs'"
}

stop_ros_stack() {
    log "stopping live navigation stack"
    tmux kill-session -t record-livox 2>/dev/null || true
    tmux kill-session -t navigation 2>/dev/null || true

    local patterns=(
        'bin/ros2 launch rmcs-navigation'
        '/livox_ros_driver2/livox_ros_driver2_node'
        '/point_lio/pointlio_mapping'
        '/rmcs_local_map/local_map'
        '/rmcs_localization/rmcs_localization'
    )
    local i pattern alive
    for i in $(seq 1 10); do
        alive=0
        for pattern in "${patterns[@]}"; do
            if pgrep -f "$pattern" >/dev/null 2>&1; then
                alive=1
                break
            fi
        done
        if [ "$alive" -eq 0 ]; then
            return 0
        fi
        sleep 1
    done

    for pattern in "${patterns[@]}"; do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done
    sleep 2
    for pattern in "${patterns[@]}"; do
        pkill -KILL -f "$pattern" 2>/dev/null || true
    done
}

cleanup() {
    if [ -n "$play_pid" ] && kill -0 "$play_pid" 2>/dev/null; then
        kill -TERM "$play_pid" 2>/dev/null || true
    fi
    if [ -n "$lio_pid" ] && kill -0 "$lio_pid" 2>/dev/null; then
        kill -INT "$lio_pid" 2>/dev/null || true
        for _ in $(seq 1 5); do
            kill -0 "$lio_pid" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$lio_pid" 2>/dev/null || true
    fi
    if [ -n "$marker" ]; then
        rm -f "$marker"
    fi
    if [ "$stack_stopped" -eq 1 ]; then
        restart_navigation
    fi
}
trap cleanup EXIT

set_status "running"
log "record-map started"

bag_dir="$(cat "$bag_state" 2>/dev/null || true)"
if [ -z "$bag_dir" ] && tmux has-session -t record-livox 2>/dev/null; then
    for _ in $(seq 1 10); do
        sleep 1
        bag_dir="$(cat "$bag_state" 2>/dev/null || true)"
        [ -n "$bag_dir" ] && break
    done
fi
if [ -z "$bag_dir" ] || [ ! -d "$bag_dir" ]; then
    log "no record bag found (state: $bag_state)"
    set_status "failed: no record bag"
    exit 1
fi
bag_name="$(basename "$bag_dir")"

if tmux has-session -t record-livox 2>/dev/null; then
    log "stopping recorder: $bag_dir"
    tmux send-keys -t record-livox:bag C-c 2>/dev/null || true
fi

for _ in $(seq 1 30); do
    [ -f "$bag_dir/metadata.yaml" ] && break
    sleep 1
done
tmux kill-session -t record-livox 2>/dev/null || true

if [ ! -f "$bag_dir/metadata.yaml" ]; then
    log "bag metadata not found: $bag_dir"
    set_status "failed: bag metadata not found"
    exit 1
fi

bag_info="$(ros2 bag info "$bag_dir" 2>&1)"
bag_info_status=$?
if [ "$bag_info_status" -ne 0 ]; then
    log "ros2 bag info failed: $bag_info"
    set_status "failed: ros2 bag info failed"
    exit 1
fi

message_count="$(printf '%s\n' "$bag_info" | awk '/^Messages:/ {print $2; exit}')"
if ! [[ "${message_count:-}" =~ ^[0-9]+$ ]] || [ "$message_count" -eq 0 ]; then
    log "bag has no messages: $bag_dir"
    set_status "failed: empty bag"
    exit 1
fi

lid_topic="$(printf '%s\n' "$bag_info" | grep -oE '/livox/lidar_[^ |]+' | head -n 1)"
imu_topic="$(printf '%s\n' "$bag_info" | grep -oE '/livox/imu_[^ |]+' | head -n 1)"
if [ -z "$lid_topic" ] || [ -z "$imu_topic" ]; then
    log "livox lidar/imu topics not found in bag: $bag_dir"
    set_status "failed: livox topics not found"
    exit 1
fi

log "bag=$bag_dir messages=$message_count lidar=$lid_topic imu=$imu_topic"

stop_ros_stack
stack_stopped=1

pcd_dir=/tmp/point-lio
mkdir -p "$pcd_dir"
marker="$(mktemp /tmp/record-map-marker.XXXXXX)"
mapper_ns="/record_mapper_$$"

config_file="$(ros2 pkg prefix point_lio)/share/point_lio/config/mid360.yaml"
if [ ! -f "$config_file" ]; then
    log "point_lio config not found: $config_file"
    set_status "failed: point_lio config not found"
    exit 1
fi

point_lio_exe="$(ros2 pkg prefix point_lio)/lib/point_lio/pointlio_mapping"
if [ ! -x "$point_lio_exe" ]; then
    log "point_lio executable not found: $point_lio_exe"
    set_status "failed: point_lio executable not found"
    exit 1
fi

log "starting point_lio in namespace $mapper_ns"
RCUTILS_COLORIZED_OUTPUT=0 "$point_lio_exe" --ros-args \
    -r __ns:="$mapper_ns" \
    --params-file "$config_file" \
    -p common.lid_topic:="$lid_topic" \
    -p common.imu_topic:="$imu_topic" \
    >"$point_lio_log" 2>&1 &
lio_pid=$!

service_ready=0
for _ in $(seq 1 60); do
    services="$(ros2 service list --no-daemon --spin-time 1 2>/dev/null || true)"
    if grep -qx "$mapper_ns/save_pcd_map" <<<"$services"; then
        service_ready=1
        break
    fi
    if ! kill -0 "$lio_pid" 2>/dev/null; then
        break
    fi
    sleep 1
done
if [ "$service_ready" -ne 1 ]; then
    log "point_lio save service not ready"
    set_status "failed: point_lio not ready"
    exit 1
fi

log "playing bag: $bag_dir"
ros2 bag play "$bag_dir" &
play_pid=$!
wait "$play_pid"
play_status=$?
play_pid=""
if [ "$play_status" -ne 0 ]; then
    log "bag playback failed with status $play_status"
    set_status "failed: bag playback failed"
    exit 1
fi
sleep 5

log "requesting pcd save"
ros2 service call "$mapper_ns/save_pcd_map" std_srvs/srv/Trigger "{}" || true

saved_pcd=""
for _ in $(seq 1 300); do
    if grep -q 'Map has been saved to:' "$point_lio_log" 2>/dev/null; then
        saved_pcd="$(grep 'Map has been saved to:' "$point_lio_log" | grep -oE '/[^[:space:]]+\.pcd' | tail -n 1)"
        break
    fi
    if ! kill -0 "$lio_pid" 2>/dev/null; then
        break
    fi
    sleep 1
done

if [ -n "$saved_pcd" ] && [ -f "$saved_pcd" ] && [ -s "$saved_pcd" ]; then
    maps_dir="$(dirname "$bag_dir")/maps"
    mkdir -p "$maps_dir"
    dest="$maps_dir/$bag_name.pcd"
    cp -f "$saved_pcd" "$dest.tmp.$$"
    mv -f "$dest.tmp.$$" "$dest"
    log "map saved: $dest"
    set_status "success $dest"
else
    log "no valid pcd produced (reported: ${saved_pcd:-none})"
    set_status "failed: no valid pcd produced"
    exit 1
fi
