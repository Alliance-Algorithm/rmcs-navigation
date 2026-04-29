local blackboard = require("blackboard").singleton()
local action = require("action")

--- 过起伏路段任务。
---
--- 流程：
---  1. 切换底盘为 step_down 模式
---  2. 发送云台角度（垂直于路段方向）
---  3. 关闭 local_map 避障
---  4. 直线导航到终点（只走前进方向）
---  5. 恢复底盘为 auto 模式
---  6. 释放云台方向覆盖
---  7. 恢复 local_map 避障
---
--- @param forward_center boolean -- true = begin→final 方向, false = final→begin
--- @param disable_obstacle? boolean -- 是否关闭 local_map 避障，默认 true
return function(forward_center, disable_obstacle)
	if disable_obstacle == nil then
		disable_obstacle = true
	end

	local user = blackboard.user
	local rule = blackboard.rule

	local begin_ours = rule.rough_terrain_begin.ours
	local begin_them = rule.rough_terrain_begin.them
	local final_ours = rule.rough_terrain_final.ours
	local final_them = rule.rough_terrain_final.them

	-- 离哪一个入口近就用哪个阵营的坐标
	local dist_ours = math.sqrt(
		(user.x - begin_ours.x) ^ 2 + (user.y - begin_ours.y) ^ 2
	)
	local dist_them = math.sqrt(
		(user.x - begin_them.x) ^ 2 + (user.y - begin_them.y) ^ 2
	)

	local begin, final
	if dist_ours <= dist_them then
		begin = begin_ours
		final = final_ours
	else
		begin = begin_them
		final = final_them
	end

	local from, to
	if forward_center then
		from = begin
		to = final
	else
		from = final
		to = begin
	end

	-- 1. 切换底盘为 step_down 模式
	action:update_chassis_mode("step_down")

	-- 2. 设定云台朝向（垂直于路段方向）
	local dx = to.x - from.x
	local dy = to.y - from.y
	local perpendicular_yaw = math.atan(dy, dx)
	action:update_gimbal_direction(perpendicular_yaw)

	-- 3. 关闭 local_map 避障
	if disable_obstacle then
		action:set_local_obstacle(false)
	end

	-- 4. 直线导航到终点（只走前进方向）
	action:navigate_straight(to, 0.3, 20, 1.8)

	-- 5. 恢复底盘为 auto 模式
	action:update_chassis_mode("auto")

	-- 6. 释放云台方向覆盖
	action:update_gimbal_direction(0 / 0)
	if disable_obstacle then
		action:set_local_obstacle(true)
	end
end
