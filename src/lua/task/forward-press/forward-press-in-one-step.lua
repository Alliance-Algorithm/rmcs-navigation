local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- 前压至对方半场起伏路段终点
--- @return boolean is_success
return function()
	action:info("开始forward-press-in-one-step")

	local rule = blackboard.rule
	local enemy_fluctuant_road_final = rule.fluctuant_road_final.them

	action:update_chassis_mode("SPIN")
	local ok = navigate_to_point(enemy_fluctuant_road_final, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn(string.format(
			"forward-press-in-one-step: 导航到对方起伏路段终点失败 (x=%.2f, y=%.2f)",
			enemy_fluctuant_road_final.x,
			enemy_fluctuant_road_final.y
		))
		return false
	end

	return true
end
