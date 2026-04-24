local blackboard = require("blackboard").singleton()
local clock = require("util.clock")
local request = require("util.scheduler").request
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

local function distance_to(target)
	local dx = target.x - blackboard.user.x
	local dy = target.y - blackboard.user.y
	return math.sqrt(dx * dx + dy * dy)
end

--- 中央高地巡航：在“靠近公路侧”与“靠近狗洞侧”之间按固定周期切换导航目标。
--- @param ours_zone boolean
--- @param switch_interval number 切换周期（秒）
return function(ours_zone, switch_interval)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(switch_interval) == "number", "switch_interval should be a number")
	assert(switch_interval > 0, "switch_interval should be positive")
	action:info("开始cruise-in-central-highlands")

	local rule = blackboard.rule
	local navigation_timeout = math.max(10.0, switch_interval * 2.0)
	local near_crossing_road, near_doghole
	if ours_zone then
		near_crossing_road = rule.central_highland_near_crossing_road.ours
		near_doghole = rule.central_highland_near_doghole.ours
	else
		near_crossing_road = rule.central_highland_near_crossing_road.them
		near_doghole = rule.central_highland_near_doghole.them
	end

	-- 首次优先去更近的点，减少无效折返。
	local go_crossing_road_first = distance_to(near_crossing_road) <= distance_to(near_doghole)
	local target = go_crossing_road_first and near_crossing_road or near_doghole

	while true do
		local phase_start = clock:now()
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

		if target == near_crossing_road then
			target = near_doghole
		else
			target = near_crossing_road
		end
	end

	return true
end
