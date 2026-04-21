local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	action:info("开始occupy-fortress")

	local rule = blackboard.rule
	local fortress
	if ours_zone then
		fortress = rule.fortress.ours
	else
		fortress = rule.fortress.them
	end

	local is_success = navigate_to_point(fortress, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not is_success then
		action:warn("前往堡垒点超时")
		return false
	end

	return true
end
