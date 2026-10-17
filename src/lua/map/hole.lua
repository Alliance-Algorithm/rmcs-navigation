local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 }),
	kStep1 = Map:point("kStep1", { x = 2.6, y = -0.4 }),
	kStep2 = Map:point("kStep2", { x = 0.4, y = -1.5 }),
	kHoleBegin = Map:point("kHoleBegin", { x = 7.3, y = -1.9 }),
	kHoleEnd = Map:point("kHoleEnd", { x = 8.3, y = -1.9 }),
	kEnd = Map:point("kEnd", { x = 9.1, y = -0.9 }),
}

local function navigate(from, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local distance = math.sqrt((to.x - from.x) ^ 2 + (to.y - from.y) ^ 2)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 20 + distance / 2,
	}
	if timeout then
		action:fuck("navigate timeout, current x=" .. bb.user.x .. " y=" .. bb.user.y)
	end
	return not timeout
end

-- 穿越地洞任务：起终点连线方向即隧道方向的 world yaw
local function cross_tunnel(from, to)
	action:cancel_target()
	return action:blocking_cross_tunnel(math.atan(to.y - from.y, to.x - from.x))
end

Map:connect(Points.kOrigin, Points.kStep1) { navigate, navigate }
Map:connect(Points.kStep1, Points.kStep2) { navigate, navigate }
Map:connect(Points.kStep2, Points.kHoleBegin) { navigate, navigate }
Map:connect_tunnel(Points.kHoleBegin, Points.kHoleEnd) { cross_tunnel, cross_tunnel }
Map:connect(Points.kHoleEnd, Points.kEnd) { navigate, navigate }

return {
	map = Map,
	points = Points,
}
