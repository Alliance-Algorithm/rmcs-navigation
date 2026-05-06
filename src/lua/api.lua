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
--- @field send_target fun(x: number, y: number)
--- @field update_gimbal_direction fun(angle: number)
--- @field update_gimbal_dominator fun(name: string)
--- @field update_chassis_mode fun(mode: string)
--- @field update_chassis_vel fun(x: number, y: number)
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

	-- FIXME: 存在调试用的进程(foxglove)，记得去掉
	local command = [[
        source %q

        screen -S rmcs-navigation -X quit

        screen -dmS rmcs-navigation
        screen -S rmcs-navigation -X screen bash -lc "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

        configs=%q
        screen -S rmcs-navigation -X screen bash -lc "ros2 launch rmcs-navigation motion.launch.yaml $configs"
        screen -S rmcs-navigation -X screen bash -lc "ros2 launch rmcs-navigation sensor.launch.yaml $configs"
    ]]
	local packed_command = string.format(command, filename, configs)

	return util.run(string.format("(%s) >/dev/null 2>&1 &", packed_command))
end

function api.stop_navigation()
	local command = [[
        screen -S rmcs-navigation -X quit
    ]]
	return util.run(string.format("(%s) >/dev/null 2>&1 &", command))
end

return api
