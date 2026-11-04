local Map = require("map.core").new()
local action = require("action")
local clock = require("util.clock")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local kLocalMapIgnoreTimeout = 5 -- 秒，忽略 local_map 的看门狗超时
local kLocalMapIgnoreRefresh = 2 -- 秒，跨越期间刷新看门狗超时的周期
local kCrossSpeedScale = 1.3 -- 忽略局部地图穿越时的导航速度增益

local kHoleTolerance = 0.15 -- 米，狗洞两端到点容差：入口要对准，出口要确实出洞后才展开云台
local kHoleEntryTimeout = 5 -- 秒，精确到达狗洞入口的时限
local kEngageRelease = 2 -- 秒，自瞄丢失目标持续该时长后结束停车迎战

local Points = {
	-- 出发点：装配区（工作台）右侧，设为 world 原点 (0, 0)
	kBegin = Map:point("kBegin", { x = 0.000, y = 0.000 }),
	kBorn = Map:point("kBorn", { x = 0.150, y = -0.600 }),
	kHoleBegin = Map:point("kHoleBegin", { x = 1.100, y = -0.070 }),
	kHoleEnd = Map:point("kHoleEnd", { x = 3.700, y = -0.070 }),
	kEscape = Map:point("kEscape", { x = 1.000, y = -5.000 }),
	-- 中央控制区四角，向内缩 0.4 m（车体半径 0.26 m），顺时针 1 -> 2 -> 3 -> 4
	kPatrol1 = Map:point("kPatrol1", { x = 3.800, y = -2.500 }),
	kPatrol2 = Map:point("kPatrol2", { x = 6.000, y = -2.500 }),
	kPatrol3 = Map:point("kPatrol3", { x = 6.000, y = -4.700 }),
	kPatrol4 = Map:point("kPatrol4", { x = 3.800, y = -4.700 }),
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
			return bb.condition.near(to, kHoleTolerance)
		end,
		timeout = 20 + distance / 2,
	}
	if timeout then
		action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	end
	return not timeout
end

-- 普通赶路：bb.context.engage_on_route 为 true 时，途中自瞄接管即停车、开小陀螺迎战，
-- 目标丢失 kEngageRelease 秒后恢复赶路；停车时长不计入超时
local function rough_navigate(from, to)
	local distance = math.sqrt((to.x - from.x) ^ 2 + (to.y - from.y) ^ 2)
	local deadline = clock:now() + 20 + distance / 2

	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	while true do
		request:wait_until {
			monitor = function()
				return bb.condition.near(to, 0.5) or (bb.context.engage_on_route and bb.autoaim.should_control)
			end,
			timeout = math.max(deadline - clock:now(), 0),
		}
		if bb.condition.near(to, 0.5) then
			return true
		end
		if clock:now() >= deadline then
			action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
			return false
		end

		action:info("途中识别到敌方装甲板，停车迎战")
		local engage_begin = clock:now()
		action:cancel_target()
		action:switch_motion_mode("attack")

		local last_seen = clock:now()
		request:wait_until {
			monitor = function()
				if bb.autoaim.should_control then
					last_seen = clock:now()
				end
				return clock:now() - last_seen >= kEngageRelease
			end,
		}

		action:info("目标丢失，继续前往 " .. to.name)
		action:switch_motion_mode("normal")
		deadline = deadline + clock:now() - engage_begin
		action:navigate(to)
	end
end

-- 钻狗洞：精确到达洞口 -> 原地对正 -> 折叠云台 -> 忽略 local_map 导航到出口 -> 恢复。
-- 超时会由 C++ 侧 watchdog 兜底恢复 local_map，避免状态残留。
-- 期间置 bb.context.crossing，endpoint 据此推迟意图切换，避免中途取消导致云台停在折叠位。
local function cross_hole(from, to)
	bb.context.crossing = true

	if not action:navigate_until(from, kHoleTolerance, kHoleEntryTimeout) then
		action:fuck("failed to reach hole entry, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		bb.context.crossing = false
		return false
	end
	action:cancel_target()

	-- 底盘为正方形，blocking_align_chassis 会选择最近的 π/2 等价朝向。
	local world_yaw = math.atan(to.y - from.y, to.x - from.x)
	if not action:blocking_align_chassis(world_yaw) then
		action:fuck("failed to align chassis, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		bb.context.crossing = false
		return false
	end

	action:blocking_set_gimbal_fold(true)
	action:update_supercap_boost(true)

	if not action:set_local_map_ignored(true, kLocalMapIgnoreTimeout, kCrossSpeedScale) then
		action:fuck("failed to ignore local map, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		action:blocking_set_gimbal_fold(false)
		action:update_supercap_boost(false)
		bb.context.crossing = false
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

	bb.context.crossing = false
	return success
end

-- ==================== rough_navigate ====================

-- 出击：kBegin -> kHoleBegin -> 狗洞 -> kHoleEnd -> 1 号点
Map:connect(Points.kBegin, Points.kHoleBegin) { rough_navigate, rough_navigate }
Map:connect_ignore_local_map(Points.kHoleBegin, Points.kHoleEnd) { cross_hole, cross_hole }
Map:connect(Points.kHoleEnd, Points.kPatrol1) { rough_navigate, rough_navigate }

-- 中央控制区巡逻：1 -> 2 -> 3 -> 4 -> 1（顺时针）
Map:connect(Points.kPatrol1, Points.kPatrol2) { rough_navigate, rough_navigate }
Map:connect(Points.kPatrol2, Points.kPatrol3) { rough_navigate, rough_navigate }
Map:connect(Points.kPatrol3, Points.kPatrol4) { rough_navigate, rough_navigate }
Map:connect(Points.kPatrol4, Points.kPatrol1) { rough_navigate, rough_navigate }

-- 返回：任意巡逻点 -> 4 号点 -> kEscape -> kBorn，不走狗洞
-- 4 号点在中央控制区南侧，接 kEscape 不穿高地；狗洞在全局地图是空地，kHoleEnd 不能直连 kEscape
Map:connect(Points.kPatrol4, Points.kEscape) { rough_navigate, rough_navigate }
Map:connect(Points.kEscape, Points.kBorn) { rough_navigate, rough_navigate }

-- 出发点与补给点之间的短接（kBegin 在装配区右侧，kBorn 在启动区中心）
Map:connect(Points.kBegin, Points.kBorn) { rough_navigate, rough_navigate }

-- 补给后再次出击：kBorn -> kHoleBegin
Map:connect(Points.kBorn, Points.kHoleBegin) { rough_navigate, rough_navigate }

bb.context.current = Points.kBegin

return {
	map = Map,
	points = Points,
}
