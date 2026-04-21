local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")
local crossing_road_zone = require("task.crossing-road-zone")

--- 开始巡航：
--- 1) 从补给点移动到公路区起点
--- 2) 正向通过公路区
--- 3) 完成持续巡航前的机动准备
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")

	local rule = blackboard.rule
	local resupply
	local road_begin
	if ours_zone then
		resupply = rule.resupply_zone.ours
		road_begin = rule.road_zone_begin.ours
	else
		resupply = rule.resupply_zone.them
		road_begin = rule.road_zone_begin.them
	end

	local ok = navigate_to_point(resupply, {
		tolerance = 0.08,
		timeout = 10,
	})
	if not ok then
		action:warn("start-cruise: 导航到补给点失败（超时）")
		return false
	end

	ok = navigate_to_point(road_begin, {
		tolerance = 0.15,
		timeout = 10,
	})
	if not ok then
		action:warn("start-cruise: 导航到公路区起点失败（超时）")
		return false
	end

	ok = crossing_road_zone(ours_zone, true)
	if not ok then
		action:warn("start-cruise: 通过公路区导航失败")
		return false
	end

	action:info("start-cruise: 已通过公路区，准备进入持续巡航")
	return true
end
