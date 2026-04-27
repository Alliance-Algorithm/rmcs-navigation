local blackboard = require("blackboard").singleton()
local clock = require("util.clock")
local request = require("util.scheduler").request
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

local switch_interval = 10.0

local function distance_to(target)
	local dx = target.x - blackboard.user.x
	local dy = target.y - blackboard.user.y
	return math.sqrt(dx * dx + dy * dy)
end

--- 在基地前方左右增益点之间按固定周期切换巡航。
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	action:info("开始cruise-in-front-of-base")

	local rule = blackboard.rule
	local left_gain_point, right_gain_point
	if ours_zone then
		left_gain_point = rule.base_left_gain_point.ours
		right_gain_point = rule.base_right_gain_point.ours
	else
		left_gain_point = rule.base_left_gain_point.them
		right_gain_point = rule.base_right_gain_point.them
	end

	local navigation_timeout = math.max(10.0, switch_interval * 2.0)
	local target
	if distance_to(left_gain_point) <= distance_to(right_gain_point) then
		target = left_gain_point
	else
		target = right_gain_point
	end

	while true do
		local phase_start = clock:now()
		action:update_chassis_mode("SPIN")
		local ok = navigate_to_point(target, {
			tolerance = 0.1,
			timeout = navigation_timeout,
		})
		if not ok then
			action:warn(string.format(
				"cruise-in-front-of-base: 导航到巡航点失败 (x=%.2f, y=%.2f, timeout=%.2fs)",
				target.x,
				target.y,
				navigation_timeout
			))
			return false
		end

		local elapsed = clock:now() - phase_start
		local remain = switch_interval - elapsed
		if remain > 0 then
			request:sleep(remain)
		end

		if target == left_gain_point then
			target = right_gain_point
		else
			target = left_gain_point
		end
	end

	return true
end
