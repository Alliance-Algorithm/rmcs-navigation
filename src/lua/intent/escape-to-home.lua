local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- 回家补给：从当前位置直接导航至补给点。
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")

	local rule = blackboard.rule
	local resupply_zone
	if ours_zone then
		resupply_zone = rule.resupply_zone.ours
	else
		resupply_zone = rule.resupply_zone.them
	end

	local is_success = navigate_to_point(resupply_zone, {
		tolerance = 0.15,
		timeout = 10,
	})
	if not is_success then
		action:warn("escape-to-home: 导航到补给点失败（超时）")
		return false
	end

	action:info("escape-to-home: 已抵达补给点")
	return true
end
