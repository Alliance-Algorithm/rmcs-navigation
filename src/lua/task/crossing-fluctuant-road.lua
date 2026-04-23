local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- @param ours_zone boolean
--- @param forward_center boolean
--- @return boolean is_success
return function(ours_zone, forward_center)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(forward_center) == "boolean", "forward_center should be a boolean")
	action:info("开始crossing-fluctuant-road")

	local rule = blackboard.rule
	local begin, final
	if ours_zone then
		begin = rule.fluctuant_road_begin.ours
		final = rule.fluctuant_road_final.ours
	else
		begin = rule.fluctuant_road_begin.them
		final = rule.fluctuant_road_final.them
	end

	local from, to
	if forward_center then
		from = begin
		to = final
	else
		from = final
		to = begin
	end

	local ok = navigate_to_point(from, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn(string.format(
			"crossing-fluctuant-road: 导航到起点失败 (x=%.2f, y=%.2f)",
			from.x,
			from.y
		))
		return false
	end

	ok = navigate_to_point(to, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn(string.format(
			"crossing-fluctuant-road: 导航到终点失败 (x=%.2f, y=%.2f)",
			to.x,
			to.y
		))
		return false
	end

	return true
end
