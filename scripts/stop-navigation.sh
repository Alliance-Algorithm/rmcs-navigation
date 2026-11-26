#!/usr/bin/env bash

# Stop the ROS navigation/sensor stack; leave the RMCS executor and remote control running.
printf '[record-map] stopping navigation and sensors\n'
tmux kill-session -t record-livox 2>/dev/null || true
tmux kill-session -t record-sensor 2>/dev/null || true
tmux kill-session -t navigation 2>/dev/null || true

patterns=(
    'bin/ros2 launch rmcs-navigation'
    '/livox_ros_driver2/livox_ros_driver2_node'
    '/point_lio/pointlio_mapping'
    '/rmcs_local_map/local_map'
    '/rmcs_localization/rmcs_localization'
)
for i in $(seq 1 10); do
    alive=0
    for pattern in "${patterns[@]}"; do
        if pgrep -f "$pattern" >/dev/null 2>&1; then
            alive=1
            break
        fi
    done
    if [ "$alive" -eq 0 ]; then
        exit 0
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
