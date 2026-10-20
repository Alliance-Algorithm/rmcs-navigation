local Map = require("map.core").new()
local action = require("action")
local clock = require("util.clock")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local kLocalMapIgnoreTimeout = 5 -- 秒，忽略 local_map 的看门狗超时
local kLocalMapIgnoreRefresh = 2 -- 秒，跨越期间刷新看门狗超时的周期

local Points = {
	kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 }),
	kHole1Begin = Map:point("kHole1Begin", { x = 3.5, y = -0.4 }),
	kHole1End = Map:point("kHole1End", { x = 3.6, y = -2.3 }),
	kHole2Begin = Map:point("kHole2Begin", { x = 7.5, y = -1.9 }),
	kHole2End = Map:point("kHole2End", { x = 8.5, y = -1.9 }),
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
			return bb.condition.near(to, 0.5)
		end,
		timeout = 20 + distance / 2,
	}
	if timeout then
		action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	end
	return not timeout
end

-- 忽略局部地图跨越：先原地对正到边方向，再折头，然后让导航忽略 local_map，
-- 正常导航到目标点，最后恢复。超时会由 C++ 侧 watchdog 兜底恢复，避免状态残留。
local function cross_ignore_local_map(from, to)
	action:cancel_target()

	-- 先原地对正，避免忽略局部地图后盲走时底盘朝向不确定。
	-- 底盘为正方形，blocking_align_chassis 会选择最近的 π/2 等价朝向。
	local world_yaw = math.atan(to.y - from.y, to.x - from.x)
	if not action:blocking_align_chassis(world_yaw) then
		action:fuck("failed to align chassis, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		return false
	end

	action:blocking_set_gimbal_fold(true)
	action:update_supercap_boost(true)

	if not action:set_local_map_ignored(true, kLocalMapIgnoreTimeout) then
		action:fuck("failed to ignore local map, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		action:blocking_set_gimbal_fold(false)
		action:update_supercap_boost(false)
		return false
	end

	-- 等待一个局部代价地图更新周期，确保障碍已被清除
	request:sleep(0.2)

	local success = navigate(from, to, function()
		if not action:set_local_map_ignored(true, kLocalMapIgnoreTimeout) then
			action:fuck("lost local map ignore gate during crossing")
		end
	end)
	action:set_local_map_ignored(false, 0)
	action:update_supercap_boost(false)
	action:blocking_set_gimbal_fold(false)
	return success
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
Map:connect_tunnel(Points.kHole1Begin, Points.kHole1End) {
	cross_ignore_local_map,
	cross_ignore_local_map,
}
Map:connect(Points.kHole1End, Points.kHole2Begin) { navigate, navigate }
Map:connect_ignore_local_map(Points.kHole2Begin, Points.kHole2End) {
	cross_ignore_local_map,
	cross_ignore_local_map,
}
Map:connect(Points.kHole2End, Points.kEnd) { navigate, navigate }

return {
	map = Map,
	points = Points,
	cross_ignore_local_map = cross_ignore_local_map,
}
