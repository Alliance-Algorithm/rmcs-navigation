local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_true = test_util.assert_true
local assert_table_eq = test_util.assert_table_eq

local calls = {
	apply_navigation_goal = {},
	restart_navigation = {},
	update_chassis_vel = {},
}

package.loaded["api"] = {
	info = function(_) end,
	warn = function(_) end,
	apply_navigation_goal = function(x, y)
		calls.apply_navigation_goal[#calls.apply_navigation_goal + 1] = { x, y }
	end,
	restart_navigation = function(config)
		calls.restart_navigation[#calls.restart_navigation + 1] = config
		return true, "ok"
	end,
	update_chassis_mode = function(_) end,
	update_gimbal_direction = function(_) end,
	update_chassis_vel = function(x, y)
		calls.update_chassis_vel[#calls.update_chassis_vel + 1] = { x, y }
	end,
}

package.loaded["blackboard"] = nil
package.loaded["main"] = nil
local Blackboard = require("blackboard")
local bb = Blackboard.singleton()

bb.play.rswitch = "DOWN"
bb.meta.timestamp = 0

require("main")

local function get_main_cache()
	for index = 1, 32 do
		local name, value = debug.getupvalue(on_init, index)
		if name == nil then
			break
		end
		if name == "cache" then
			return value
		end
	end
	error("failed to find cache upvalue from main.on_init")
end

on_init()
local cache = get_main_cache()

local function tick(now)
	bb.meta.timestamp = now
	on_tick()
end

tick(0)
assert_eq(#calls.apply_navigation_goal, 0, "initial tick should not send NaN goal")

cache.goal.x = 1.5
cache.goal.y = -2.0

tick(0.1)
assert_eq(#calls.apply_navigation_goal, 1, "goal change should trigger immediate send")
assert_table_eq(calls.apply_navigation_goal[1], { 1.5, -2.0 }, "immediate goal payload")

tick(1.0)
assert_eq(#calls.apply_navigation_goal, 1, "without new goal change should not resend before period")

tick(2.0)
assert_eq(#calls.apply_navigation_goal, 2, "periodic task should resend goal every 2 seconds")
assert_table_eq(calls.apply_navigation_goal[2], { 1.5, -2.0 }, "periodic goal payload")

assert_eq(#calls.restart_navigation, 0, "edge callback should not trigger on DOWN")
bb.play.rswitch = "UP"
tick(2.1)
assert_eq(#calls.restart_navigation, 1, "rising edge should trigger restart once")
assert_eq(calls.restart_navigation[1], "rmul", "restart config")

tick(2.2)
assert_eq(#calls.restart_navigation, 1, "holding UP should not retrigger restart")

control_speed_callback(0.3, -0.4, 42.0)
assert_eq(#calls.update_chassis_vel, 1, "control_speed_callback should forward chassis velocity")
assert_table_eq(calls.update_chassis_vel[1], { 0.3, -0.4 }, "chassis velocity payload")

assert_true(true, "runable smoke test finished")

print("runable.lua: ok")
