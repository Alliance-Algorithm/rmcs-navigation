local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kOrigin = Map:point("kBegin", { x = 0.0, y = 0.0 }),
	kAttackInLeft = Map:point("kAttackInLeft", { x = 2.0, y = 3.5 }),
	kAttackInRight = Map:point("kAttackInRight", { x = 2.4, y = 0.2 }),
	kStepBegin = Map:point("kStepBegin", { x = 2.4, y = -0.7 }),
	kStepFinal = Map:point("kStepFinal", { x = 2.4, y = -2.0 }),
	kUnderSlope = Map:point("kUnderSlope", { x = 4.7, y = -2.6 }),
	kUnderRune = Map:point("kUnderRune", { x = 4.6, y = 1.1 }),
	kAttackAtHole = Map:point("kAttackAtHole", { x = 5.3, y = 3.8 }),
}

local function navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 15,
	}
	return not timeout
end

Map:connect(Points.kOrigin, Points.kAttackInLeft) { navigate, navigate }
Map:connect(Points.kOrigin, Points.kAttackInRight) { navigate, navigate }
Map:connect(Points.kAttackInRight, Points.kStepBegin) { navigate, navigate }
Map:connect(Points.kStepBegin, Points.kStepFinal) {
	function(from, to)
		return action:blocking_cross_step(math.atan(to.y - from.y, to.x - from.x), true)
	end,
	function(from, to)
		return action:blocking_cross_step(math.atan(to.y - from.y, to.x - from.x), false)
	end,
}
Map:connect(Points.kStepFinal, Points.kUnderSlope) { navigate, navigate }
Map:connect(Points.kUnderSlope, Points.kUnderRune) { navigate, navigate }
Map:connect(Points.kUnderRune, Points.kAttackAtHole) { navigate, navigate }

return {
	map = Map,
	points = Points,
}
