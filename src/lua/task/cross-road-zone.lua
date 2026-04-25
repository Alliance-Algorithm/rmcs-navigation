local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- @param ours_zone boolean
--- @param forward_center boolean
--- @return boolean is_success
return function(ours_zone, forward_center)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(forward_center) == "boolean", "forward_center should be a boolean")
	action:info("开始cross-road-zone")

	local rule = blackboard.rule
	local road_begin, road_final
	if ours_zone then
		road_begin = rule.road_zone_begin.ours
		road_final = rule.road_zone_final.ours
	else
		road_begin = rule.road_zone_begin.them
		road_final = rule.road_zone_final.them
	end

	local from, to
	if forward_center then
		from = road_begin
		to = road_final
	else
		from = road_final
		to = road_begin
	end

	local ok = navigate_to_point(from, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("cross-road-zone: 导航到公路区起点失败")
		return false
	end

	ok = navigate_to_point(to, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("cross-road-zone: 导航到公路区终点失败")
		return false
	end

	return true
end
