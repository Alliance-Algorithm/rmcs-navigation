local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- @param ours_zone boolean
--- @param use_begin boolean
--- @return boolean is_success
return function(ours_zone, use_begin)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(use_begin) == "boolean", "use_begin should be a boolean")

	local rule = blackboard.rule
	local point
	if ours_zone then
		point = use_begin and rule.fluctuant_road_begin.ours or rule.fluctuant_road_final.ours
	else
		point = use_begin and rule.fluctuant_road_begin.them or rule.fluctuant_road_final.them
	end

	return navigate_to_point(point, {
		tolerance = 0.1,
		timeout = 10,
	})
end
