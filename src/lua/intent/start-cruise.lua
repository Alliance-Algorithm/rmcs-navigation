local action = require("action")
local crossing_road_zone = require("task.crossing-road-zone")

--- 开始巡航：
--- 1) 从当前位置直接进入公路区跨越流程
--- 2) 正向通过公路区
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")

	local ok = crossing_road_zone(ours_zone, true)
	if not ok then
		action:warn("start-cruise: 通过公路区导航失败")
		return false
	end

	action:info("start-cruise: 已通过公路区，准备进入持续巡航")
	return true
end
