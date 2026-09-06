local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 }),
	-- kStep1 = Map:point("kStep1", { x = -2.8, y = 0.1 }),
	kStep2 = Map:point("kStep2", { x = -3.0, y = 1.0 }),
	-- kStep3 = Map:point("kStep3", { x = -0.5, y = 1.1 }),
	kStep4 = Map:point("kStep4", { x = -0.8, y = 2.2 }),
	-- kStep5 = Map:point("kStep5", { x = -5.0, y = 2.2 }),
	kLand = Map:point("kLand", { x = -4.8, y = -1.8 }),
	-- kStep6 = Map:point("kStep6", { x = -5.3, y = -3.5 }),
	-- kStep7 = Map:point("kStep7", { x = -0.9, y = -2.9 }),
}

local function navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 20,
	}
	action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	return not timeout
end

-- Map:connect(Points.kOrigin, Points.kStep1) { navigate, navigate }
-- Map:connect(Points.kStep1, Points.kStep2) { navigate, navigate }
Map:connect(Points.kOrigin, Points.kStep2) { navigate, navigate }

Map:connect(Points.kStep2, Points.kStep4) { navigate, navigate }
-- Map:connect(Points.kStep3, Points.kStep4) { navigate, navigate }
-- Map:connect(Points.kStep4, Points.kStep5) { navigate, navigate }
Map:connect(Points.kStep4, Points.kLand) { navigate, navigate }
-- Map:connect(Points.kLand, Points.kStep6) { navigate, navigate }
-- Map:connect(Points.kStep6, Points.kStep7) { navigate, navigate }
-- Map:connect(Points.kStep7, Points.kOrigin) { navigate, navigate }
Map:connect(Points.kLand, Points.kOrigin) { navigate, navigate }


return {
	map = Map,
	points = Points,
}
