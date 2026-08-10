local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	-- 己方半场

	kOrigin = Map:point("kBegin", { x = 0.0, y = 0.0 }),
	kHome = Map:point("kHome", { x = -1.8, y = -5.0 }),

	kOriginLeft = Map:point("kOriginLeft", { x = 0.0, y = 1.6 }),

	kSelfFortressRight = Map:point("kSelfFortressRight", { x = 2.5, y = -2.0 }),

	kSelfHighlandBegin = Map:point("kSelfHighlandBegin", { x = 2.5, y = 2.0 }),
	kSelfHighlandFinal = Map:point("kSelfHighlandFinal", { x = 2.5, y = 4.0 }),

	kAttackRune = Map:point("kAttackRune", { x = 5.1, y = 2.1 }),
	kAttackOutpost = Map:point("kAttackOutpost", { x = 7.9, y = 5.3 }),

	kBehindOutpost = Map:point("kBehindOutpost", { x = 6.2, y = -3.9 }),
	kSelfStepBegin = Map:point("kSelfStepBegin", { x = 4.45, y = -4.3 }),
	kSelfStepFinal = Map:point("kSelfStepFinal", { x = 4.45, y = -6.3 }),

	kSelfSlopeBegin = Map:point("kSelfSlopeBegin", { x = 7.0, y = -6.4 }),
	kThemSlopeFinal = Map:point("kThemSlopeFinal", { x = 8.3, y = 7.0 }),

	kSelfDoubleStepsBegin = Map:point("kSelfDoubleStepsBegin", { x = 4.9, y = 0.4 }),
	kSelfDoubleStepsFinal = Map:point("kSelfDoubleStepsFinal", { x = 7.5, y = 0 }),

	-- 中央高地

	kNearSelfOutpost = Map:point("kNearSelfOutpost", { x = 10.4, y = -4 }),
	kNearThemOutpost = Map:point("kNearThemOutpost", { x = 10.4, y = 4 }),

	-- 对方半场

	kThemDoubleStepsBegin = Map:point("kThemDoubleStepsBegin", { x = 15.7, y = -0.2 }),
	kThemDoubleStepsFinal = Map:point("kThemDoubleStepsFinal", { x = 13.3, y = 0 }),

	kThemStepBegin = Map:point("kThemStepBegin", { x = 16.2, y = 4.3 }),
	kThemStepFinal = Map:point("kThemStepFinal", { x = 16.2, y = 6.3 }),

	kThemHighlandBegin = Map:point("kThemHighlandBegin", { x = 17.5, y = -1.6 }),
	kThemHighlandFinal = Map:point("kThemHighlandFinal", { x = 17.5, y = -3.7 }),

	kThemThigh = Map:point("kThemThigh", { x = 17.5, y = -6.6 }),

	kAttackBase = Map:point("kAttackBase", { x = 23.4, y = -1.6 }),

	kAttackBaseFront = Map:point("kAttackBaseFront", { x = 20.0, y = 0.0 }),
}

-- 临时测试：坐标缩小
-- for _, p in pairs(Points) do
--  p.x = p.x * 0.2
--  p.y = p.y * 0.2
-- end

local function rough_navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 10,
	}
	if timeout then
		action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	end
	return not timeout
end

-- 爬坡任务：need_boost 为 true（上坡）时开启超级电容，下坡不开
local function cross_slope(need_boost)
	return function(_, to)
		if need_boost then
			action:update_supercap_boost(true)
		end
		action:navigate(to)
		action:info("cross slope to " .. to.x .. ", " .. to.y)
		local timeout = request:wait_until {
			monitor = function()
				return bb.condition.near(to, 0.3)
			end,
			timeout = 10,
		}
		if timeout then
			action:fuck("cross slope timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
		end
		if need_boost then
			action:update_supercap_boost(false)
		end
		return not timeout
	end
end

-- 跨越地形任务：正向 from -> to，反向 to -> from
local function cross_step(is_forward)
	return function(from, to)
		action:navigate(from)
		request:wait_until {
			monitor = function()
				return bb.condition.near(to, 0.15)
			end,
			timeout = 5,
		}
		action:cancel_target()

		action:update_supercap_boost(true)
		local success = action:blocking_cross_step(math.atan(to.y - from.y, to.x - from.x), is_forward)
		action:update_supercap_boost(false)
		return success
	end
end

-- ==================== rough_navigate ====================

-- 出生区
Map:connect(Points.kOrigin, Points.kHome) { rough_navigate, rough_navigate }
Map:connect(Points.kOrigin, Points.kOriginLeft) { rough_navigate, rough_navigate }
Map:connect(Points.kOrigin, Points.kSelfFortressRight) { rough_navigate, rough_navigate }
Map:connect(Points.kOriginLeft, Points.kSelfHighlandBegin) { rough_navigate, rough_navigate }

-- 己方上路（高地 - 符点 - 前哨）
Map:connect(Points.kSelfHighlandBegin, Points.kAttackRune) { rough_navigate, rough_navigate }
Map:connect(Points.kAttackRune, Points.kAttackOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfHighlandFinal, Points.kThemSlopeFinal) { rough_navigate, rough_navigate }

-- 己方下路（双台阶 - 台阶 - 坡道）
Map:connect(Points.kSelfHighlandBegin, Points.kSelfDoubleStepsBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfDoubleStepsBegin, Points.kSelfFortressRight) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfDoubleStepsBegin, Points.kSelfStepBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfStepBegin, Points.kSelfFortressRight) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfStepBegin, Points.kBehindOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfStepFinal, Points.kSelfSlopeBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfSlopeBegin, Points.kNearSelfOutpost) { rough_navigate, rough_navigate }

-- 中央区（双台阶终点 - 两个前哨观察位构成的环）
Map:connect(Points.kSelfDoubleStepsFinal, Points.kNearSelfOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kNearSelfOutpost, Points.kThemDoubleStepsFinal) { rough_navigate, rough_navigate }
Map:connect(Points.kThemDoubleStepsFinal, Points.kNearThemOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kNearThemOutpost, Points.kSelfDoubleStepsFinal) { rough_navigate, rough_navigate }

-- 对方半场
Map:connect(Points.kNearThemOutpost, Points.kThemStepFinal) { rough_navigate, rough_navigate }
Map:connect(Points.kThemStepBegin, Points.kThemDoubleStepsBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kThemDoubleStepsBegin, Points.kThemHighlandBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kThemHighlandFinal, Points.kThemThigh) { rough_navigate, rough_navigate }
Map:connect(Points.kThemHighlandBegin, Points.kAttackBase) { rough_navigate, rough_navigate }

-- ==================== cross_slope（爬坡） ====================

Map:connect(Points.kSelfHighlandBegin, Points.kSelfHighlandFinal) { cross_slope(true), cross_slope(false) }
Map:connect(Points.kThemHighlandBegin, Points.kThemHighlandFinal) { cross_slope(true), cross_slope(false) }

Map:connect(Points.kSelfStepBegin, Points.kSelfStepFinal) { cross_step(true), cross_step(false) }
Map:connect(Points.kThemStepBegin, Points.kThemStepFinal) { cross_step(true), cross_step(false) }

Map:connect(Points.kAttackBaseFront, Points.kAttackBase) { rough_navigate, rough_navigate }
Map:connect(Points.kAttackBaseFront, Points.kThemHighlandBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kThemStepBegin, Points.kAttackBaseFront) { rough_navigate, rough_navigate }

bb.context.current = Points.kOrigin

local M = {
	map = Map,
	points = Points,
}

return M
