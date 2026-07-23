local util = require("util.native")

---
--- Cxx Impl
---

--- @class Api
---
--- @field switch_topic_forward fun(enable: boolean)
---
--- @field info fun(message: string)
--- @field warn fun(message: string)
--- @field fuck fun(message: string)
---
--- @field update_enable_control fun(enable: boolean)
--- @field send_target fun(x: number, y: number)
--- @field update_gimbal_direction fun(yaw: number, pitch: number)
--- @field switch_motion_mode fun(mode: "normal" | "attack" | "road" | "step" | "slope")
--- @field update_under_attack fun(yes: boolean)
--- @field relocalize fun() 触发重定位，红/蓝方由 referee robot_id 自动派生
---
local api = setmetatable({}, {
	__index = function(_, name)
		return function(...)
			local args = {}
			for i = 1, select("#", ...) do
				args[i] = tostring(select(i, ...))
			end
			print(string.format("[api stub] %s(%s)", name, table.concat(args, ", ")))
		end
	end,
})

---
--- Native Impl
---

--- @param config { launch_livox: boolean, launch_odin1: boolean, global_map: string, use_sim_time: boolean }
function api.restart_navigation(config)
	config = config or {}

	local filename, msg = util.search_setup_resource()
	if not filename then
		error(msg)
	end

	local launch_livox = tostring(config.launch_livox or "false")
	local launch_odin1 = tostring(config.launch_odin1 or "false")
	local global_map = tostring(config.global_map or "empty")
	local use_sim_time = tostring(config.use_sim_time or "false")

	local configs = string.format(
		"launch_livox:=%s launch_odin1:=%s global_map:=%s use_sim_time:=%s",
		launch_livox,
		launch_odin1,
		global_map,
		use_sim_time
	)

	local template = [[
        source %q

        # 杀死已存在的 navigation 会话（忽略错误）
        tmux kill-session -t navigation 2>/dev/null

        # 传入配置参数
        configs=%q

        # 创建后台会话并启动 motion (窗口 0)
        tmux new-session -d -s navigation -n "motion" "bash -lc 'ros2 launch rmcs-navigation motion.launch.yaml $configs'"

        # 创建新窗口启动 sensor (窗口 1)
        tmux new-window -t navigation -n "sensor" "bash -lc 'ros2 launch rmcs-navigation sensor.launch.yaml $configs'"
    ]]
	local command = string.format(template, filename, configs)

	return util.run(string.format("(%s) >/dev/null 2>&1 &", command))
end

function api.stop_navigation()
	local command = [[
        tmux kill-session -t navigation 2>/dev/null || true
    ]]
	return util.run(string.format("(%s) >/dev/null 2>&1 &", command))
end

function api.start_record()
	local filename, msg = util.search_setup_resource()
	if not filename then
		error(msg)
	end

	local template = [[
        source %q

        tmux kill-session -t record-livox 2>/dev/null || true

        tmux new-session -d -s record-livox -n "bag" "bash -lc 'source %q

        if [ -d /home/ubuntu ]; then
            record_parent=/home/ubuntu
        elif [ -d /root ]; then
            record_parent=/root
        else
            echo \"record parent not found\"
            exit 1
        fi
        echo \"Bag will be saved to ${record_parent}\"

        mapfile -t topics < <(ros2 topic list | rg \"^/livox/(lidar|imu)_[^/]+$\")
        if [ \${#topics[@]} -eq 0 ]; then
            echo \"no /livox lidar/imu topics found\"
            exit 1
        fi

        record_root=\$record_parent/record-livox
        bag_dir=\$record_root/livox-$(TZ=Asia/Shanghai date +%%Y_%%m_%%d-%%H_%%M_%%S)
        mkdir -p \"\$record_root\"

        ros2 bag record -o \"\$bag_dir\" \"\${topics[@]}\"'"
    ]]
	local command = string.format(template, filename, filename)

	return util.run(string.format("(%s) >/dev/null 2>&1 &", command))
end

function api.abort_record()
	local command = [[
        if tmux has-session -t record-livox 2>/dev/null; then
            tmux send-keys -t record-livox:bag C-c
            sleep 2
            tmux kill-session -t record-livox 2>/dev/null || true
        fi
    ]]
	return util.run(string.format("(%s) >/dev/null 2>&1 &", command))
end

function api.toggle_record()
	local command = [[tmux has-session -t record-livox 2>/dev/null]]
	local ok = util.run(command)
	if ok then
		return api.abort_record()
	end

	return api.start_record()
end

return api
