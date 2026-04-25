local blackboard = require("blackboard").singleton()
local clock = require("util.clock")
local request = require("util.scheduler").request
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- 前压至对方高地在二级台阶侧与高地增益点之间巡航。
--- @param switch_interval number 切换周期（秒）
--- @return boolean is_success
return function(switch_interval)
	assert(type(switch_interval) == "number", "switch_interval should be a number")
	assert(switch_interval > 0, "switch_interval should be positive")
	action:info("开始forward-press-in-two-step")

	local rule = blackboard.rule
	local enemy_gain_point = rule.central_highland_gain_pount.them
	local enemy_near_two_steps_and_outpost = rule.central_highland_near_two_steps_and_outpost.them

	local navigation_timeout = math.max(10.0, switch_interval * 2.0)
	local target = enemy_gain_point

	while true do
		local phase_start = clock:now()
		action:update_chassis_mode("SPIN")
		ok = navigate_to_point(target, {
			tolerance = 0.1,
			timeout = navigation_timeout,
		})
		if not ok then
			action:warn(string.format(
				"forward-press-in-two-step: 导航到巡航点失败 (x=%.2f, y=%.2f, timeout=%.2fs)",
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

		if target == enemy_gain_point then
			target = enemy_near_two_steps_and_outpost
		else
			target = enemy_gain_point
		end
	end

	return true
end
