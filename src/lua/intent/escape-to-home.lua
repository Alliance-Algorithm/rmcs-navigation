local blackboard = require("blackboard").singleton()
local action = require("action")
local go_down_onestep = require("task.one-step.go-down-onestep")
local cross_fluctuant_road = require("task.cross-fluctuant.cross-fluctuant-road")
local navigate_to_point = require("task.navigate-to-point")

--- @param route "direct"|"onestep"|"fluctuant_road"
--- @return boolean is_success
return function(route)
	assert(type(route) == "string", "route should be a string")

	local resupply_zone = blackboard.rule.resupply_zone.ours
	local is_success = true

	if route == "onestep" then
		action:info("escape-to-home: 开跟随走下台阶路线回家")
		is_success = go_down_onestep(true)
		if not is_success then
			action:warn("escape-to-home: 下一级台阶失败")
			return false
		end
	elseif route == "fluctuant_road" then
		action:info("escape-to-home: 走起伏路路线回家")
		is_success = cross_fluctuant_road(true, false)
		if not is_success then
			action:warn("escape-to-home: 通过起伏路失败")
			return false
		end
	elseif route == "direct" then
		action:info("escape-to-home: 走直接回家路线")
	else
		action:warn("unknown escape route: " .. tostring(route))
	end

	action:update_chassis_mode("SPIN")
	is_success = navigate_to_point(resupply_zone, {
		tolerance = 0.15,
		timeout = 10,
	})
	if not is_success then
		action:warn("escape-to-home: 导航到补给点失败")
		return false
	end

	action:info("escape-to-home: 已抵达补给点")
	return true
end
