local blackboard = require("blackboard").singleton()
local action = require("action")
local Map = require("map")
local request = require("util.scheduler").request
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
		action:warn("go-down-onestep: 导航到一级台阶高点失败")
		return false
	end

	action:update_chassis_mode("LAUNCH_RAMP")
	local dx = one_step_low.x - one_step_high.x
	local dy = one_step_low.y - one_step_high.y
	local distance = math.sqrt(dx * dx + dy * dy)
	if distance <= 0 then
		action:warn("go-down-onestep: 一级台阶高点与低点重合，无法确定下台阶方向")
		return false
	end

	local gimbal_yaw = math.atan(dy, dx)
	local linear_speed = 2.0
	local vx = dx / distance * linear_speed
	local vy = dy / distance * linear_speed
	action:info(string.format(
		"go-down-onestep: 云台朝向=%.3f rad, 底盘速度=(%.3f, %.3f) m/s",
		gimbal_yaw,
		vx,
		vy
	))
	action:update_gimbal_direction(gimbal_yaw)

	action:update_chassis_vel(vx, vy)
	request:sleep(1.0)
	local map = Map.singleton()
	local expected_region = ours_zone and Map.Region.OURS_ROAD_TO_FLUCTUANT
		or Map.Region.THEM_ROAD_TO_FLUCTUANT
	ok = map:locate({
		x = blackboard.user.x,
		y = blackboard.user.y,
	}) == expected_region
	action:update_chassis_vel(0.0, 0.0)
	if not ok then
		action:warn("go-down-onestep: 导航到一级台阶低点失败")
		return false
	end
	action:update_chassis_mode("SPIN")

	return true
end
