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
--- @field update_gimbal_direction fun(angle: number)
--- @field update_gimbal_dominator fun(name: string)
--- @field switch_controller fun(mode: "normal" | "road" | "step" | "slope")
--- @field exchange_17mm_bullet fun(amount: integer)
--- @field switch_mode fun(mode: number)
--- @field confirm_revive fun()
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

        # 创建后台会话并启动 foxglove (窗口 0)
        tmux new-session -d -s navigation -n "foxglove" "bash -lc 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'"

        # 传入配置参数
        configs=%q

        # 创建新窗口启动 motion (窗口 1)
        tmux new-window -t navigation -n "motion" "bash -lc 'ros2 launch rmcs-navigation motion.launch.yaml $configs'"

        # 创建新窗口启动 sensor (窗口 2)
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

return api
