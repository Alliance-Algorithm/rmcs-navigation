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
	local filename, msg = util.find_env_setup_bash()
	if not filename then
		error(msg)
	end

	local launch_livox = tostring(config.launch_livox)
	local launch_odin1 = tostring(config.launch_odin1)
	local global_map = tostring(config.global_map)
	local use_sim_time = tostring(config.use_sim_time)

	local sensor_config = string.format(
		"launch_livox:=%s launch_odin1:=%s global_map:=%s use_sim_time:=%s",
		launch_livox,
		launch_odin1,
		global_map,
		use_sim_time
	)
	local motion_config = string.format("use_sim_time:=%s", use_sim_time)

	local command = [[
        source %q
        screen -S rmcs-navigation -X quit 2>/dev/null

        screen -dmS rmcs-navigation
        screen -S rmcs-navigation -X screen bash -lc "ros2 launch rmcs-navigation sensor.launch.yaml %s"
        screen -S rmcs-navigation -X screen bash -lc "ros2 launch rmcs-navigation motion.launch.yaml %s"
    ]]
	local packed_command = string.format(command, filename, sensor_config, motion_config)

	util.run_command(packed_command)
end

return api
