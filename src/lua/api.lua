local util = require("util.native")

--- @class Api
---
--- @field info fun(message: string)
--- @field warn fun(message: string)
--- @field fuck fun(message: string)
---
--- @field apply_navigation_goal fun(x: number, y: number)
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

function api.restart_navigation(config)
	local screen_label = "rmcs-navigation"
	if config == nil or config == "" then
		error("api.restart_navigation(config): config is required and cannot be empty")
	end

	local target_label = config
	local env_setup_bash, env_error = util.search_setup_filename()
	if env_setup_bash == nil then
		error("api.restart_navigation(config): " .. tostring(env_error))
	end

	local launch_command = string.format(
		"source %q && ros2 launch rmcs-navigation online.launch.py config_name:=%q",
		env_setup_bash,
		target_label
	)

	return util.run_command(
		string.format(
			"if screen -S %q -Q select . >/dev/null 2>&1; then screen -S %q -X quit >/dev/null 2>&1; fi; sleep 0.5; screen -dmS %q bash -lc %q",
			screen_label,
			screen_label,
			screen_label,
			launch_command
		)
	)
end

return api
