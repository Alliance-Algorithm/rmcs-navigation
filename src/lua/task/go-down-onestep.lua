local blackboard = require("blackboard").singleton()
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

--- 从当前位置依次经过一级台阶高点与低点。
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	action:info("开始go-down-onestep")

	local rule = blackboard.rule
	local one_step_high, one_step_low
	if ours_zone then
		one_step_high = rule.one_step_begin.ours
		one_step_low = rule.one_step_final.ours
	else
		one_step_high = rule.one_step_begin.them
		one_step_low = rule.one_step_final.them
	end

	local ok = navigate_to_point(one_step_high, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("go-down-onestep: 导航到一级台阶高点失败（超时）")
		return false
	end

	ok = navigate_to_point(one_step_low, {
		tolerance = 0.1,
		timeout = 10,
	})
	if not ok then
		action:warn("go-down-onestep: 导航到一级台阶低点失败（超时）")
		return false
	end

	return true
end
