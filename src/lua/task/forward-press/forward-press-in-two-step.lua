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
	local enemy_gain_point = rule.central_highland_gain_point.them
	local enemy_near_two_steps_and_outpost = rule.central_highland_near_two_steps_and_outpost.them

	local navigation_timeout = math.max(10.0, switch_interval * 2.0)
	local targets = {
		{
			name = "central_highland_gain_point",
			point = enemy_gain_point,
		},
		{
			name = "central_highland_near_two_steps_and_outpost",
			point = enemy_near_two_steps_and_outpost,
		},
	}
	local target_index = 1

	while true do
		local target = targets[target_index]
		local phase_start = clock:now()
		action:update_chassis_mode("SPIN")
		local ok = navigate_to_point(target.point, {
			tolerance = 0.1,
			timeout = navigation_timeout,
		})
		if not ok then
			action:warn(string.format(
				"forward-press-in-two-step: 导航到%s失败 (x=%.2f, y=%.2f, timeout=%.2fs)",
				target.name,
				target.point.x,
				target.point.y,
				navigation_timeout
			))
			return false
		end

		local elapsed = clock:now() - phase_start
		local remain = switch_interval - elapsed
		if remain > 0 then
			request:sleep(remain)
		end

		target_index = target_index % #targets + 1
	end

	return true
end
