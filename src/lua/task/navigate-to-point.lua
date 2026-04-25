local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request
local action = require("action")

--- @class NavigateToPointOptions
--- @field tolerance? number|{x: number, y: number}
--- @field timeout? number

local function normalize_options(options)
	options = options or {}
	assert(type(options) == "table", "options should be a table")

	local tolerance = options.tolerance or 0.1
	if type(tolerance) == "number" then
		assert(tolerance >= 0, "tolerance should be non-negative")
	else
		assert(type(tolerance) == "table", "tolerance should be number or {x, y}")
		assert(type(tolerance.x) == "number", "tolerance.x should be a number")
		assert(type(tolerance.y) == "number", "tolerance.y should be a number")
		assert(tolerance.x >= 0 and tolerance.y >= 0, "tolerance.{x,y} should be non-negative")
	end

	local timeout = options.timeout or 10
	assert(type(timeout) == "number", "timeout should be a number")
	assert(timeout >= 0, "timeout should be non-negative")

	return tolerance, timeout
end

--- 普通点位导航：设置目标点并等待到达（或超时）。
--- @param point {x: number, y: number}
--- @param options? NavigateToPointOptions
--- @return boolean is_success
return function(point, options)
	assert(type(point) == "table", "point should be a table")
	assert(type(point.x) == "number", "point.x should be a number")
	assert(type(point.y) == "number", "point.y should be a number")
	action:info("开始navigate-to-point")

	local tolerance, timeout = normalize_options(options)
	local condition = blackboard.condition

	action:navigate(point)
	local is_timeout = request:wait_until {
		monitor = function()
			return condition.near(point, tolerance)
		end,
		timeout = timeout,
	}
	return not is_timeout
end
