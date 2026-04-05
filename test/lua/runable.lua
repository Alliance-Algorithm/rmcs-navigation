local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local root = script_dir .. "../.."
package.path = table.concat({
	root .. "/src/lua-decision/?.lua",
	root .. "/src/lua-decision/?/init.lua",
	root .. "/src/lua-decision/?/?.lua",
	package.path,
}, ";")

local function assert_eq(actual, expected, message)
	if actual ~= expected then
		error(string.format("%s: expected %s, got %s", message, tostring(expected), tostring(actual)))
	end
end

local function assert_table_eq(actual, expected, message)
	assert_eq(#actual, #expected, message .. " length")
	for i = 1, #expected do
		assert_eq(actual[i], expected[i], message .. string.format("[%d]", i))
	end
end

local calls = {
	restart_navigation = {},
	update_goal = {},
}

package.loaded["api"] = {
	restart_navigation = function(config)
		calls.restart_navigation[#calls.restart_navigation + 1] = config
		return true, "ok"
	end,
	update_goal = function(position)
		calls.update_goal[#calls.update_goal + 1] = { position[1], position[2] }
	end,
	update_gimbal_direction = function(_) end,
	update_chassis_mode = function(_) end,
}

package.loaded["main"] = nil
local Blackboard = require("blackboard")
local bb = Blackboard.singleton()

bb.user.health = 100
bb.user.bullet = 100
bb.game.stage = "RUNNING"
bb.play.rswitch = "DOWN"
bb.play.lswitch = "UNKNOWN"
bb.rule.decision = "auxiliary"
bb.rule.health_limit = 50
bb.rule.health_ready = 80
bb.rule.bullet_limit = 10
bb.rule.bullet_ready = 20
bb.rule.home = { 1.5, -2.0 }
bb.meta.timestamp = 0

require("main")

on_init()

on_tick()
assert_eq(#calls.restart_navigation, 0, "restart should not trigger before edge")

bb.play.rswitch = "UP"
bb.meta.timestamp = 1
on_tick()
assert_eq(#calls.restart_navigation, 1, "restart should trigger on UP rising edge")
assert_eq(calls.restart_navigation[1], "rmul", "restart config")

bb.meta.timestamp = 2
on_tick()
assert_eq(#calls.restart_navigation, 1, "restart should not retrigger while staying UP")

bb.play.rswitch = "DOWN"
bb.meta.timestamp = 3
on_tick()

bb.play.rswitch = "UP"
bb.meta.timestamp = 4
on_tick()
assert_eq(#calls.restart_navigation, 2, "restart should retrigger on the next UP rising edge")

bb.user.health = 20
bb.meta.timestamp = 5
on_tick()
assert_eq(#calls.update_goal, 1, "low health should send robot home once")
assert_table_eq(calls.update_goal[1], bb.rule.home, "home goal")

bb.meta.timestamp = 6
on_tick()
assert_eq(#calls.update_goal, 1, "waiting condition should not resend goal")

bb.user.health = 90
bb.meta.timestamp = 7
on_tick()
assert_eq(#calls.update_goal, 1, "health ready should finish without extra goals")

print("runable.lua: ok")
