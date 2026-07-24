local Map = require("map.core").new()
local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local Points = {
	kA = Map:point("A", { x = 0.0, y = 0.0 }),
	kB = Map:point("B", { x = 0.2, y = 1.0 }),
	kC = Map:point("C", { x = 1.0, y = 2.5 }),
	kD = Map:point("D", { x = 2.5, y = 0.0 }),
	kE = Map:point("E", { x = 2.55, y = -0.8 }),
	kF = Map:point("F", { x = 2.4, y = -2.1 }),
	kG = Map:point("G", { x = 5.2, y = -2.2 }),
	kH = Map:point("H", { x = 5.2, y = 1.2 }),
	kI = Map:point("I", { x = 5.1, y = 2.7 }),
	kJ = Map:point("J", { x = 5.1, y = 3.7 }),
}

local function navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.3)
		end,
	}
	bb.context.current = to
end

local function navigate_with_boost(_, to)
	action:set_boost(true)
	navigate(_, to)
	action:set_boost(false)
end

local function face_direction(from, to)
	local dx = to.x - from.x
	local dy = to.y - from.y
	local yaw = math.atan(dy, dx)
	action:chassis_direction(yaw)
	action:gimbal_toward(yaw, 0)
end

local kClimbTimeout = 15
local kClimbMaxRetries = 2

local function climb_with_retry(from, to)
	for attempt = 1, kClimbMaxRetries do
		action:switch_motion_mode("normal")
		face_direction(from, to)
		request:sleep(1.5)
		action:switch_motion_mode("climb")
		local timed_out = request:wait_until {
			monitor = function()
				return bb.user.auto_climb_success
			end,
			timeout = kClimbTimeout,
		}
		if timed_out then
			action:warn(string.format(
				"climb attempt %d/%d timed out, returning to %s",
				attempt, kClimbMaxRetries, from.name))
			action:chassis_direction_free()
			action:switch_motion_mode("normal")
			request:sleep(1)
			action:navigate(from)
			request:wait_until {
				monitor = function()
					return bb.condition.near(from, 0.2)
				end,
			}
		else
			action:info("climb succeeded, navigating to " .. to.name)
			action:chassis_direction_free()
			action:navigate(to)
			request:yield()
			request:sleep(1.0)
			action:switch_motion_mode("normal")
			request:wait_until {
				monitor = function()
					return bb.condition.near(to, 0.3)
				end,
			}
			bb.context.current = to
			return
		end
	end
	action:chassis_direction_free()
	bb.user.climb_failed = true
end

Map:connect(Points.kA, Points.kB) { navigate, navigate }
Map:connect(Points.kB, Points.kC) { navigate, navigate }
Map:connect(Points.kA, Points.kD) { navigate, navigate }
Map:connect(Points.kD, Points.kE) { navigate, navigate }
Map:connect(Points.kE, Points.kF) {
	function(_, to)
		climb_with_retry(Points.kE, to)
	end,
	function(_, to)
		action:switch_motion_mode("normal")
		face_direction(Points.kF, Points.kE)
		request:sleep(1.5)
		action:switch_motion_mode("support_arm")
		local timed_out = request:wait_until {
			monitor = function()
				return bb.user.support_arm_success
			end,
			timeout = 30,
		}
		action:chassis_direction_free()
		if timed_out then
			error("support arm failed")
		end
		action:navigate(to)
		request:yield()
		request:sleep(1.0)
		action:switch_motion_mode("normal")
		request:wait_until {
			monitor = function()
				return bb.condition.near(to, 0.3)
			end,
		}
		bb.context.current = to
	end,
}
Map:connect(Points.kF, Points.kG) { navigate, navigate }
Map:connect(Points.kG, Points.kH) { navigate_with_boost, navigate }
Map:connect(Points.kH, Points.kI) { navigate, navigate }
Map:connect(Points.kI, Points.kJ) { navigate, navigate_with_boost }

return {
	map = Map,
	points = Points,
}
