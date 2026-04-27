local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- @return boolean is_success
return function()
	action:info("开始occupy-fortress")

	local rule = blackboard.rule
	local fortress = rule.fortress.ours

	action:update_chassis_mode("SPIN")
	local is_success = navigate_to_point(fortress, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not is_success then
		action:warn(string.format(
			"occupy-fortress: 导航到己方堡垒失败 (x=%.2f, y=%.2f)",
			fortress.x,
			fortress.y
		))
		return false
	end

	return true
end
