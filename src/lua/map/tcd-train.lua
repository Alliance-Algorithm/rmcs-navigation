local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 }),
	kStep1 = Map:point("kStep1", { x = -2.1, y = -0.1 }),
	kStep2 = Map:point("kStep2", { x = -2.3, y = 0.9 }),
	kStep3 = Map:point("kStep3", { x = -0.7, y = 1.1 }),
	kStep4 = Map:point("kStep4", { x = -0.9, y = 2.1 }),
	kStep5 = Map:point("kStep5", { x = -5.2, y = 1.9 }),
	kLand = Map:point("kLand", { x = -4.8, y = -2.2 }),
	kStep6 = Map:point("kStep6", { x = -4.9, y = -3.5 }),
	kStep7 = Map:point("kStep7", { x = -1.2, y = -2.6 }),
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
	action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	return not timeout
end

Map:connect(Points.kOrigin, Points.kStep1) { navigate, navigate }
Map:connect(Points.kStep1, Points.kStep2) { navigate, navigate }
Map:connect(Points.kStep2, Points.kStep3) { navigate, navigate }
Map:connect(Points.kStep3, Points.kStep4) { navigate, navigate }
Map:connect(Points.kStep4, Points.kStep5) { navigate, navigate }
Map:connect(Points.kStep5, Points.kLand) { navigate, navigate }
Map:connect(Points.kLand, Points.kStep6) { navigate, navigate }
Map:connect(Points.kStep6, Points.kStep7) { navigate, navigate }
Map:connect(Points.kStep7, Points.kOrigin) { navigate, navigate }

return {
	map = Map,
	points = Points,
}
