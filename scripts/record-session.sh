#!/usr/bin/env bash

set -o pipefail

env_file=$1
state_dir=$2
bag_pid=""
final_status=""
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$state_dir" || exit 1
exec >>"$state_dir/record-session.log" 2>&1
exec 9>"$state_dir/record.lock"
flock -n 9 || exit 1

set_status() {
    printf '%s\n' "$*" >"$state_dir/record.status.tmp"
    mv -f "$state_dir/record.status.tmp" "$state_dir/record.status"
}

stop_bag() {
    [ -n "$bag_pid" ] || return 0
    local result
    # Non-interactive Bash background jobs inherit SIGINT ignored. ros2 bag
    # explicitly handles SIGTERM by stopping the recorder and flushing storage.
    kill -TERM "$bag_pid" 2>/dev/null || true
    for _ in $(seq 1 300); do
        kill -0 "$bag_pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$bag_pid" 2>/dev/null; then
        printf '[record-session] recorder shutdown timed out; forcing exit\n'
        kill -KILL "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
        bag_pid=""
        return 1
    fi
    wait "$bag_pid" 2>/dev/null
    result=$?
    bag_pid=""
    if [ "$result" -ne 0 ]; then
        printf '[record-session] recorder exited with status %s\n' "$result"
    fi
    return "$result"
}

cleanup() {
    local result=$?
    stop_bag || true
    if ! bash "$script_dir/stop-navigation.sh"; then
        result=1
        final_status="failed: navigation shutdown failed"
    fi
    if [ "$result" -ne 0 ] && [[ "$final_status" != failed:* ]]; then
        final_status="failed: record workflow exited ($result)"
    fi
    [ -z "$final_status" ] || set_status "$final_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

fail() {
    local message="$*"
    final_status="failed: ${message#failed: }"
    exit 1
}

source "$env_file" || fail "cannot load ROS environment"
set_status starting
rm -f "$state_dir/last-record-bag"

topics="$(ros2 topic list --no-daemon --spin-time 1 2>/dev/null)"
if ! grep -qE '^/livox/lidar_[^/]+$' <<<"$topics" ||
   ! grep -qE '^/livox/imu_[^/]+$' <<<"$topics"; then
    if ! pgrep -f '/livox_ros_driver2/livox_ros_driver2_node' >/dev/null; then
        printf -v driver 'bash -lc %q' "source $(printf '%q' "$env_file") && ros2 launch livox_ros_driver2 msg_MID360_launch.py"
        tmux new-session -d -s record-sensor "$driver" || fail "cannot start Livox driver"
    fi
fi

lidar_topics=()
imu_topics=()
for _ in $(seq 1 30); do
    if [ -f "$state_dir/record.stop" ] || [ -f "$state_dir/record.abort" ]; then
        final_status=aborted
        exit 0
    fi
    topics="$(ros2 topic list --no-daemon --spin-time 1 2>/dev/null)"
    mapfile -t lidar_topics < <(grep -E '^/livox/lidar_[^/]+$' <<<"$topics")
    mapfile -t imu_topics < <(grep -E '^/livox/imu_[^/]+$' <<<"$topics")
    [ "${#lidar_topics[@]}" -gt 0 ] && [ "${#imu_topics[@]}" -gt 0 ] && break
    sleep 0.2
done
[ "${#lidar_topics[@]}" -gt 0 ] && [ "${#imu_topics[@]}" -gt 0 ] || fail "Livox lidar/imu topics not ready"

if [ -n "${RMCS_RECORD_ROOT:-}" ]; then
    record_root=$RMCS_RECORD_ROOT
elif [ -d /home/ubuntu ]; then
    record_root=/home/ubuntu/record-livox
else
    record_root=/root/record-livox
fi
mkdir -p "$record_root" || fail "cannot create record directory"
bag_dir="$record_root/livox-$(TZ=Asia/Shanghai date +%Y_%m_%d-%H_%M_%S-%N)"
printf '%s\n' "$bag_dir" >"$state_dir/last-record-bag"
ros2 bag record -o "$bag_dir" "${lidar_topics[@]}" "${imu_topics[@]}" &
bag_pid=$!
storage_ready=0
for _ in $(seq 1 100); do
    kill -0 "$bag_pid" 2>/dev/null || fail "recorder did not start"
    if [ -f "$state_dir/record.stop" ] || [ -f "$state_dir/record.abort" ]; then
        final_status="aborted $bag_dir"
        exit 0
    fi
    if [ -d "$bag_dir" ] && find "$bag_dir" -maxdepth 1 -type f \
        \( -name '*.mcap' -o -name '*.db3' \) -print -quit | grep -q .; then
        storage_ready=1
        break
    fi
    sleep 0.1
done
[ "$storage_ready" -eq 1 ] || fail "recorder storage not ready"
set_status "recording $bag_dir"

while kill -0 "$bag_pid" 2>/dev/null; do
    [ -f "$state_dir/record.stop" ] && break
    [ -f "$state_dir/record.abort" ] && break
    sleep 0.1
done
if [ ! -f "$state_dir/record.stop" ] && [ ! -f "$state_dir/record.abort" ]; then
    fail "recorder exited unexpectedly"
fi
stop_bag || fail "recorder did not shut down cleanly; bag retained at $bag_dir"
if [ -f "$state_dir/record.abort" ]; then
    final_status="aborted $bag_dir"
    exit 0
fi

set_status "mapping $bag_dir"
if RMCS_RECORD_STATE_DIR="$state_dir" RMCS_RECORD_BAG="$bag_dir" RMCS_RECORD_ENV_FILE="$env_file" \
   bash "$script_dir/record-map.sh"; then
    final_status="$(cat "$state_dir/record-map.status")"
else
    fail "$(cat "$state_dir/record-map.status" 2>/dev/null || printf 'mapping failed')"
fi
