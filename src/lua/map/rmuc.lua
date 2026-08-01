local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	-- 己方半场

	kOrigin = Map:point("kBegin", { x = 0.0, y = 0.0 }),
	kHome = Map:point("kHome", { x = -1.8, y = -5.0 }),

	kSelfHighlandBegin = Map:point("kSelfHighlandBegin", { x = 2.7, y = 1.7 }),
	kSelfHighlandFinal = Map:point("kSelfHighlandFinal", { x = 2.7, y = 4.0 }),

	kAttackRune = Map:point("kAttackRune", { x = 6.6, y = 3.7 }),
	kAttackOutpost = Map:point("kAttackOutpost", { x = 7.9, y = 5.3 }),

	kBehindOutpost = Map:point("kBehindOutpost", { x = 6.2, y = -3.9 }),
	kSelfStepBegin = Map:point("kSelfStepBegin", { x = 4.5, y = -4.3 }),
	kSelfStepFinal = Map:point("kSelfStepFinal", { x = 4.5, y = -6.3 }),

	kSelfSlopeBegin = Map:point("kSelfSlopeBegin", { x = 8.2, y = -6.8 }),
	kThemSlopeFinal = Map:point("kThemSlopeFinal", { x = 8.3, y = 7 }),

	kSelfDoubleStepsBegin = Map:point("kSelfDoubleStepsBegin", { x = 4.7, y = 0 }),
	kSelfDoubleStepsFinal = Map:point("kSelfDoubleStepsFinal", { x = 7.5, y = 0 }),

	-- 中央高地

	kNearSelfOutpost = Map:point("kNearSelfOutpost", { x = 10.4, y = -4 }),
	kNearThemOutpost = Map:point("kNearThemOutpost", { x = 10.4, y = 4 }),

	-- 对方半场

	kThemDoubleStepsBegin = Map:point("kThemDoubleStepsBegin", { x = 16.0, y = 0 }),
	kThemDoubleStepsFinal = Map:point("kThemDoubleStepsFinal", { x = 13.3, y = 0 }),

	kThemStepBegin = Map:point("kThemStepBegin", { x = 16.2, y = 4.3 }),
	kThemStepFinal = Map:point("kThemStepFinal", { x = 16.2, y = 6.3 }),

	kThemHighlandBegin = Map:point("kThemHighlandBegin", { x = 17.5, y = -1.6 }),
	kThemHighlandFinal = Map:point("kThemHighlandFinal", { x = 17.5, y = -3.7 }),
}

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
		action:cancel_target()
		action:update_supercap_boost(true)
		local success = action:blocking_cross_step(math.atan(to.y - from.y, to.x - from.x), is_forward)
		action:update_supercap_boost(false)
		return success
	end
end

Map:connect(Points.kOrigin, Points.kHome) { rough_navigate, rough_navigate }
Map:connect(Points.kOrigin, Points.kSelfHighlandBegin) { rough_navigate, rough_navigate }

Map:connect(Points.kSelfHighlandFinal, Points.kThemSlopeFinal) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfHighlandBegin, Points.kSelfDoubleStepsBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfHighlandBegin, Points.kAttackRune) { rough_navigate, rough_navigate }
Map:connect(Points.kAttackRune, Points.kAttackOutpost) { rough_navigate, rough_navigate }

Map:connect(Points.kSelfDoubleStepsBegin, Points.kSelfStepBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfStepBegin, Points.kBehindOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfStepFinal, Points.kSelfSlopeBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kSelfSlopeBegin, Points.kNearSelfOutpost) { rough_navigate, rough_navigate }

Map:connect(Points.kSelfDoubleStepsFinal, Points.kNearSelfOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kNearSelfOutpost, Points.kThemDoubleStepsFinal) { rough_navigate, rough_navigate }
Map:connect(Points.kThemDoubleStepsFinal, Points.kNearThemOutpost) { rough_navigate, rough_navigate }
Map:connect(Points.kNearThemOutpost, Points.kSelfDoubleStepsFinal) { rough_navigate, rough_navigate }

Map:connect(Points.kNearThemOutpost, Points.kThemStepFinal) { rough_navigate, rough_navigate }
Map:connect(Points.kThemStepBegin, Points.kThemDoubleStepsBegin) { rough_navigate, rough_navigate }
Map:connect(Points.kThemDoubleStepsBegin, Points.kThemHighlandBegin) { rough_navigate, rough_navigate }

Map:connect(Points.kSelfHighlandBegin, Points.kSelfHighlandFinal) { cross_slope(true), cross_slope(false) }
Map:connect(Points.kThemHighlandBegin, Points.kThemHighlandFinal) { cross_slope(true), cross_slope(false) }

Map:connect(Points.kSelfStepBegin, Points.kSelfStepFinal) { cross_step(true), cross_step(false) }
Map:connect(Points.kThemStepFinal, Points.kThemStepBegin) { cross_step(true), cross_step(false) }

local M = {
	map = Map,
	points = Points,
}

return M
