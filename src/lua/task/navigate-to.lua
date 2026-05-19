local action = require("action")
local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request

--- 导航到单点。
--- @param point {x: number, y: number}
return function(point, dwell, tolerance)
	action:navigate(point)
	local timed_out = request:wait_until {
		monitor = function()
			return blackboard.condition.near(point, tolerance or 0.3)
		end,
		timeout = 30,
	}
	if timed_out then
		action:warn(string.format("navigate-to (%.2f, %.2f) 30s 内未到位，跳过 dwell", point.x, point.y))
		return false
	end
	if dwell and dwell > 0 then
		request:sleep(dwell)
	end
	return true
end
