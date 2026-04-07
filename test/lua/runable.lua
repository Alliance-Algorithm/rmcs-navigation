local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_table_eq = test_util.assert_table_eq

local calls = {
	update_goal = {},
}

package.loaded["api"] = {
	info = function(_) end,
	warn = function(_) end,
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

bb.user.health = 20
bb.meta.timestamp = 1
on_tick()
assert_eq(#calls.update_goal, 1, "low health should send robot home once")
assert_table_eq(calls.update_goal[1], bb.rule.home, "home goal")

bb.meta.timestamp = 2
on_tick()
assert_eq(#calls.update_goal, 1, "waiting condition should not resend goal")

bb.user.health = 90
bb.meta.timestamp = 3
on_tick()
assert_eq(#calls.update_goal, 1, "health ready should finish without extra goals")

print("runable.lua: ok")
