---@class Api
---@field update_goal fun(x: number, y: number)
---@field update_gimbal_direction fun(angle: number)
---@field update_chassis_mode fun(mode: string)
---@field restart_navigation fun(config: string): boolean, string

local util = require("util")
local api = {}

---@param config string
---@return boolean, string
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

	local restart_script = string.format(
		"if screen -S %q -Q select . >/dev/null 2>&1; then screen -S %q -X quit >/dev/null 2>&1; fi; sleep 0.5; screen -dmS %q bash -lc %q",
		screen_label,
		screen_label,
		screen_label,
		launch_command
	)

	local dispatch_command = string.format("bash -lc %q >/tmp/rmcs-navigation-restart.log 2>&1 &", restart_script)

	return util.run_command(dispatch_command)
end

return api
