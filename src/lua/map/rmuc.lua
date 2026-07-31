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

	-- 6.6 3.7
	kAttackRune = Map:point("kAttackRune", { x = 6.6, y = 3.7 }),
	kAttackOutpost = Map:point("kAttackOutpost", { x = 7.9, y = 5.3 }),

	kBehindOutpost = Map:point("kBehindOutpost", { x = 6.2, -3.9 }),
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

	kThemDoubleStepsBegin = Map:point("kThemDoubleStepsBegin", {}),
	kThemDoubleStepsFinal = Map:point("kThemDoubleStepsFinal", {}),

	kThemStepBegin = Map:point("kThemStepBegin", { x = 16.2, y = 4.3 }),
	kThemStepFinal = Map:point("kThemStepFinal", { x = 16.2, y = 6.3 }),

	kThemHighlandBegin = Map:point("kThemHighlandBegin", { x = 17.5, y = -1.6 }),
	kThemHighlandFinal = Map:point("kThemHighlandFinal", { x = 2.7, y = -3.7 }),
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
	action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	return not timeout
end

Map:connect(Points.kOrigin, Points.kHome) { rough_navigate, rough_navigate }

local M = {
	map = Map,
	points = Points,
}

return M
