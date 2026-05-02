local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

local function select_point(point, ours_zone)
	if type(point.x) == "number" and type(point.y) == "number" then
		return point
	end
	return ours_zone and point.ours or point.them
end

--- @param ours_zone boolean
--- @param forward_center boolean
--- @return boolean is_success
return function(ours_zone, forward_center)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(forward_center) == "boolean", "forward_center should be a boolean")
	action:info("开始cross-road-zone")

	local rule = blackboard.rule
	local targets = {
		{
			name = "road_zone_way_point_1",
			point = select_point(rule.road_zone_way_point_1, ours_zone),
		},
		{
			name = "road_zone_way_point_2",
			point = select_point(rule.road_zone_way_point_2, ours_zone),
		},
		{
			name = "road_zone_final",
			point = select_point(rule.road_zone_final, ours_zone),
		},
	}

	for _, target in ipairs(targets) do
		local ok = navigate_to_point(target.point, {
			tolerance = 0.1,
			timeout = 10,
		})
		if not ok then
			action:warn(string.format(
				"cross-road-zone: 导航到%s失败 (x=%.2f, y=%.2f)",
				target.name,
				target.point.x,
				target.point.y
			))
			return false
		end
	end

	action:update_chassis_mode("SPIN")
	return true
end
