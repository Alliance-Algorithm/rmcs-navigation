local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kA = Map:point("A", { x = 0.0, y = 0.0 }),
	kB = Map:point("B", { x = 0.0, y = 1.0 }),
	kC = Map:point("C", { x = 1.0, y = 3.0 }),
	kD = Map:point("D", { x = 2.8, y = 0.2 }),
	kE = Map:point("E", { x = 2.8, y = -0.8 }),
}

local function navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.2)
		end,
	}
	bb.context.current = to
end

Map:connect(Points.kA, Points.kB) { navigate, navigate }
Map:connect(Points.kB, Points.kC) { navigate, navigate }
Map:connect(Points.kA, Points.kD) { navigate, navigate }
Map:connect(Points.kD, Points.kE) { navigate, navigate }

return {
	map = Map,
	points = Points,
}
