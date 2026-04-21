local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- 训练专用：从当前点导航到公路区起点，再导航到公路区终点。
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	action:info("开始crossing-road-zone-train")

	local rule = blackboard.rule
	local road_begin, road_final
	if ours_zone then
		road_begin = rule.road_zone_begin.ours
		road_final = rule.road_zone_final.ours
	else
		road_begin = rule.road_zone_begin.them
		road_final = rule.road_zone_final.them
	end

	local ok = navigate_to_point(road_begin, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("crossing-road-zone-train: 导航到公路区起点失败（超时）")
		return false
	end

	ok = navigate_to_point(road_final, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("crossing-road-zone-train: 导航到公路区终点失败（超时）")
		return false
	end

	return true
end
