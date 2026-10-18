local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq

local fake = {
	status = 0.5,
	finish = {},
	direction = {},
	mode = nil,
}

package.loaded["api"] = {
	info = function() end,
	warn = function() end,
	fuck = function() end,
	set_tunnel_mode = function(mode)
		fake.mode = mode
	end,
	set_tunnel_direction = function(world_yaw)
		fake.direction[#fake.direction + 1] = world_yaw
	end,
	set_tunnel_finish = function(enable)
		fake.finish[#fake.finish + 1] = enable
	end,
	get_tunnel_status = function()
		return fake.status
	end,
}

local action = require("action")
local bb = require("blackboard").singleton()
local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()

local function assert_nan(value, message)
	assert_eq(value ~= value, true, message)
end

do
	local target = { x = 1.0, y = 0.0 }
	local result = nil

	scheduler:append_task(function()
		result = action:blocking_cross_tunnel(1.0, target, "up")
	end)

	scheduler:spin_once()
	assert_eq(fake.mode, "up", "should select tunnel mode")
	assert_eq(fake.direction[#fake.direction], 1.0, "should request tunnel direction")
	assert_eq(fake.finish[#fake.finish], false, "should clear finish request at start")

	bb.user.x = 0.85
	scheduler:spin_once()
	assert_eq(#fake.finish, 1, "should not request finish outside tolerance")

	bb.user.x = 0.95
	scheduler:spin_once()
	assert_eq(fake.finish[#fake.finish], true, "should request finish within tolerance")
	assert_eq(result, nil, "should keep waiting for tunnel status")

	fake.status = 1.0
	scheduler:spin_once()
	assert_eq(result, true, "should return success on tunnel success")
	assert_eq(fake.finish[#fake.finish], false, "should clear finish request on exit")
	assert_nan(fake.direction[#fake.direction], "should cancel tunnel direction on exit")
end

do
	fake.status = 0.5
	fake.finish = {}
	fake.direction = {}
	bb.user.x, bb.user.y = 0.0, 0.0

	local result = nil
	scheduler:append_task(function()
		result = action:blocking_cross_tunnel(0.0, { x = 1.0, y = 0.0 }, "down")
	end)

	scheduler:spin_once()
	assert_eq(fake.mode, "down", "should select tunnel mode")
	scheduler:spin_once()
	fake.status = -1.0
	scheduler:spin_once()
	assert_eq(result, false, "should return failure on tunnel failure")
end

print("action.lua: ok")
