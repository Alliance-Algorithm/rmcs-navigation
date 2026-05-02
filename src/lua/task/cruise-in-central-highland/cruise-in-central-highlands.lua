local blackboard = require("blackboard").singleton()
local clock = require("util.clock")
local request = require("util.scheduler").request
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

local function select_point(point, ours_zone)
	if type(point.x) == "number" and type(point.y) == "number" then
		return point
	end
	return ours_zone and point.ours or point.them
end

--- 中央高地巡航：按固定顺序在中央高地巡航点之间切换导航目标。
--- @param ours_zone boolean
--- @param switch_interval number 切换周期（秒）
return function(ours_zone, switch_interval)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(switch_interval) == "number", "switch_interval should be a number")
	assert(switch_interval > 0, "switch_interval should be positive")
	action:info("开始cruise-in-central-highlands")

	local rule = blackboard.rule
	local navigation_timeout = math.max(10.0, switch_interval * 2.0)
	local near_fluctuant_road, middle, near_doghole
	if ours_zone then
		near_fluctuant_road = rule.central_highland_near_fluctuant_road.ours
		near_doghole = rule.central_highland_near_doghole.ours
	else
		near_fluctuant_road = rule.central_highland_near_fluctuant_road.them
		near_doghole = rule.central_highland_near_doghole.them
	end
	middle = select_point(rule.central_highland_middle, ours_zone)

	local targets = {
		near_fluctuant_road,
		middle,
		near_doghole,
	}
	local target_index = 1

	while true do
		local target = targets[target_index]
		local phase_start = clock:now()
		action:update_chassis_mode("SPIN")
		local ok = navigate_to_point(target, {
			tolerance = 0.1,
			timeout = navigation_timeout,
		})
		if not ok then
			action:warn(string.format(
				"cruise-in-central-highlands: 导航到巡航点失败 (x=%.2f, y=%.2f, timeout=%.2fs)",
				target.x,
				target.y,
				navigation_timeout
			))
			return false
		end

		-- 保持固定切换周期：若提前到达，则驻留到本周期结束后再切点。
		local elapsed = clock:now() - phase_start
		local remain = switch_interval - elapsed
		if remain > 0 then
			request:sleep(remain)
		end

		target_index = target_index % #targets + 1
	end

	return true
end
