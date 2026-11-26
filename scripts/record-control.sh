#!/usr/bin/env bash

set -e

operation=$1
env_file=$2
state_dir=$3

case "$operation" in
    probe)
        tmux has-session -t record-workflow 2>/dev/null
        ;;
    start)
        # Adopt an independent workflow surviving an rmcs service restart.
        if tmux has-session -t record-workflow 2>/dev/null; then
            exit 0
        fi
        source "$env_file"
        mkdir -p "$state_dir"
        script_path="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/record-session.sh"
        [ -f "$script_path" ]
        rm -f "$state_dir/record.stop" "$state_dir/record.abort"
        printf 'starting\n' >"$state_dir/record.status.control"
        mv -f "$state_dir/record.status.control" "$state_dir/record.status"
        printf -v worker 'bash %q %q %q' "$script_path" "$env_file" "$state_dir"
        tmux new-session -d -s record-workflow -n record "$worker"
        ;;
    stop|abort)
        : >"$state_dir/record.$operation"
        ;;
    *)
        printf 'unknown record operation: %s\n' "$operation" >&2
        exit 1
        ;;
esac
