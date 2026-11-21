local Map = require("map.core").new()
local action = require("action")
local clock = require("util.clock")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local kLocalMapIgnoreTimeout = 5 -- 秒，忽略 local_map 的看门狗超时
local kLocalMapIgnoreRefresh = 2 -- 秒，跨越期间刷新看门狗超时的周期
local kCrossSpeedScale = 1.3 -- 忽略局部地图穿越时的导航速度增益
local kNavigateTolerance = 0.15 -- 米，到达容差；洞口需精确停在全局地图的合法格内

local Points = {
	kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 }),
	kHole1Begin = Map:point("kHole1Begin", { x = 3.5, y = -0.15 }),
	kHole1End = Map:point("kHole1End", { x = 3.5, y = -1.6 }),
	kPatrol1 = Map:point("kPatrol1", { x = 4.1, y = -1.6 }),
	kPatrol2 = Map:point("kPatrol2", { x = 4.9, y = -1.6 }),
	kHole2Begin = Map:point("kHole2Begin", { x = 7.0, y = -1.6 }),
	kHole2End = Map:point("kHole2End", { x = 9.0, y = -1.58 }),
	kEnd = Map:point("kEnd", { x = 9.1, y = -1.3 }),
}

local function navigate(from, to, keepalive)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local distance = math.sqrt((to.x - from.x) ^ 2 + (to.y - from.y) ^ 2)
	local last_keepalive = clock:now()
	local timeout = request:wait_until {
		monitor = function()
			if keepalive ~= nil and clock:now() - last_keepalive >= kLocalMapIgnoreRefresh then
				keepalive()
				last_keepalive = clock:now()
			end
			return bb.condition.near(to, kNavigateTolerance)
		end,
		timeout = 20 + distance / 2,
	}
	if timeout then
		action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		action:cancel_target()
	end
	return not timeout
end

-- 忽略局部地图跨越：上坡先停小陀螺并原地对正，再折云台直上；下坡保持小陀螺开启，
-- 无需对正。让导航忽略 local_map 走完跨越，最后恢复。超时会由 C++ 侧 watchdog 兜底恢复，
-- 避免状态残留。比赛中小陀螺由 attack 逻辑控制且全程开启，所以上坡需要临时切到 normal 停转，
-- 跨越结束后再切回 attack。
local function cross_ignore_local_map_impl(from, to, uphill)
	action:cancel_target()

	if uphill then
		-- 停小陀螺，避免一边转一边爬坡；对正后再直上。
		action:switch_motion_mode("normal")

		-- 先原地对正，避免忽略局部地图后盲走时底盘朝向不确定。
		-- 底盘为正方形，blocking_align_chassis 会选择最近的 π/2 等价朝向。
		local world_yaw = math.atan(to.y - from.y, to.x - from.x)
		if not action:blocking_align_chassis(world_yaw) then
			action:fuck("failed to align chassis, current x=" .. bb.user.x .. " y=" .. bb.user.y)
			action:switch_motion_mode("attack")
			return false
		end
	else
		-- 下坡保持小陀螺开启，直接下坡。
		action:switch_motion_mode("attack")
	end

	action:blocking_set_gimbal_fold(true)
	action:update_supercap_boost(true)

	if not action:set_local_map_ignored(true, kLocalMapIgnoreTimeout, kCrossSpeedScale) then
		action:fuck("failed to ignore local map, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		action:blocking_set_gimbal_fold(false)
		action:update_supercap_boost(false)
		action:switch_motion_mode("attack")
		return false
	end

	-- 等待一个局部代价地图更新周期，确保障碍已被清除
	request:sleep(0.2)

	local success = navigate(from, to, function()
		if not action:set_local_map_ignored(true, kLocalMapIgnoreTimeout, kCrossSpeedScale) then
			action:fuck("lost local map ignore gate during crossing")
		end
	end)
	action:set_local_map_ignored(false, 0, 1.0)
	action:update_supercap_boost(false)
	action:blocking_set_gimbal_fold(false)
	-- 比赛中保持小陀螺全程开启。
	action:switch_motion_mode("attack")
	return success
end

-- 上坡：不转小陀螺，对正后直上。
local function cross_ignore_local_map(from, to)
	return cross_ignore_local_map_impl(from, to, true)
end

-- 下坡：开启小陀螺，无需对正。
local function cross_ignore_local_map_spin(from, to)
	return cross_ignore_local_map_impl(from, to, false)
end

-- 穿越地洞任务：起终点连线方向即隧道方向的 world yaw
local function cross_tunnel(mode)
	return function(from, to)
		action:cancel_target()
		return action:blocking_cross_tunnel(
			math.atan(to.y - from.y, to.x - from.x),
			to,
			mode
		)
	end
end

Map:connect(Points.kOrigin, Points.kHole1Begin) { navigate, navigate }
-- 一号洞：正向为上坡，反向为下坡
Map:connect_ignore_local_map(Points.kHole1Begin, Points.kHole1End) {
	cross_ignore_local_map,
	cross_ignore_local_map_spin,
}
Map:connect(Points.kHole1End, Points.kPatrol1) { navigate, navigate }
Map:connect(Points.kPatrol1, Points.kPatrol2) { navigate, navigate }
Map:connect(Points.kHole1End, Points.kHole2Begin) { navigate, navigate }
-- 二号洞：正向为下坡，反向为上坡
Map:connect_ignore_local_map(Points.kHole2Begin, Points.kHole2End) {
	cross_ignore_local_map_spin,
	cross_ignore_local_map,
}
Map:connect(Points.kHole2End, Points.kEnd) { navigate, navigate }

return {
	map = Map,
	points = Points,
	cross_ignore_local_map = cross_ignore_local_map,
	cross_ignore_local_map_spin = cross_ignore_local_map_spin,
}
