local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request
local action = require("action")

--- @param ours_zone boolean
--- @param forward_center boolean
--- @return boolean is_success
return function(ours_zone, forward_center)
	action:info("开始crossing-fluctuant-road")

	local x = blackboard.user.x
	local y = blackboard.user.y

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

	local condition = blackboard.condition

	action:navigate(from)
	local is_timeout = request:wait_until {
		monitor = function()
			return condition.near(from, 0.1)
		end,
		timeout = 10,
	}

	if is_timeout then
		action:warn(string.format(
			"crossing-fluctuant-road: 导航到起点失败 (x=%.2f, y=%.2f)",
			from.x,
			from.y
		))
		return false
	end

	action:navigate(to)
	local is_timeout = request:wait_until {
		monitor = function()
			return condition.near(to, 0.1)
		end,
		timeout = 10,
	}
	if is_timeout then
		action:warn(string.format(
			"crossing-fluctuant-road: 导航到终点失败 (x=%.2f, y=%.2f)",
			to.x,
			to.y
		))
		return false
	end

	return not is_timeout
end
