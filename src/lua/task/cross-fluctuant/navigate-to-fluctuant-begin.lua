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
--- @param use_begin boolean|nil
--- @return boolean is_success
return function(ours_zone, use_begin)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	if use_begin == nil then
		use_begin = true
	end
	assert(type(use_begin) == "boolean", "use_begin should be a boolean")

	local rule = blackboard.rule
	local targets
	if use_begin then
		targets = {
			{
				name = "road_zone_begin",
				point = select_point(rule.road_zone_begin, ours_zone),
			},
			{
				name = "road_zone_way_point_1",
				point = select_point(rule.road_zone_way_point_1, ours_zone),
			},
			{
				name = "fluctuant_road_begin",
				point = select_point(rule.fluctuant_road_begin, ours_zone),
			},
		}
	else
		targets = {
			{
				name = "fluctuant_road_final",
				point = select_point(rule.fluctuant_road_final, ours_zone),
			},
		}
	end

	for _, target in ipairs(targets) do
		local ok = navigate_to_point(target.point, {
			tolerance = 0.1,
			timeout = 10,
		})
		if not ok then
			action:warn(string.format(
				"navigate-to-fluctuant-begin: 导航到%s失败 (x=%.2f, y=%.2f)",
				target.name,
				target.point.x,
				target.point.y
			))
			return false
		end
	end

	return true
end
