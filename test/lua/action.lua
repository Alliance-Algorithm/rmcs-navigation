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
	ignore_local_map = nil,
	ignore_timeout = nil,
	ignore_speed_scale = nil,
	fold_request = nil,
	fold_state = 0,
	align_direction = {},
	align_status = 0,
}

package.loaded["api"] = {
	info = function() end,
	warn = function() end,
	fuck = function() end,
	set_local_map_ignored = function(ignored, timeout, speed_scale)
		fake.ignore_local_map = ignored
		fake.ignore_timeout = timeout
		fake.ignore_speed_scale = speed_scale
		return true
	end,
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
	set_gimbal_fold = function(fold)
		fake.fold_request = fold
	end,
	get_gimbal_fold_state = function()
		return fake.fold_state
	end,
	set_chassis_align = function(world_yaw)
		fake.align_direction[#fake.align_direction + 1] = world_yaw
	end,
	get_chassis_align_status = function()
		return fake.align_status
	end,
}

local action = require("action")
local bb = require("blackboard").singleton()
local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()

local function assert_nan(value, message)
	assert_eq(value ~= value, true, message)
end

local function assert_near(actual, expected, message)
	assert_eq(type(actual), "number", message .. " type")
	assert_eq(math.abs(actual - expected) < 1e-9, true, message)
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
	local result = action:set_local_map_ignored(true, 12.5, 1.3)
	assert_eq(result, true, "set_local_map_ignored should forward api result")
	assert_eq(fake.ignore_local_map, true, "should forward ignored flag")
	assert_eq(fake.ignore_timeout, 12.5, "should forward timeout")
	assert_eq(fake.ignore_speed_scale, 1.3, "should forward speed scale")

	action:set_local_map_ignored(false, 0, 1.0)
	assert_eq(fake.ignore_local_map, false, "should forward restore flag")
	assert_eq(fake.ignore_speed_scale, 1.0, "should forward reset speed scale")
end

do
	bb.user.yaw = 0.0
	fake.align_status = 0.3
	fake.align_direction = {}

	local result = nil
	scheduler:append_task(function()
		result = action:blocking_align_chassis(1.25)
	end)

	scheduler:spin_once()
	assert_near(
		fake.align_direction[#fake.align_direction],
		1.25 - math.pi / 2,
		"should request nearest face align direction"
	)
	assert_eq(result, nil, "should wait for align status")

	scheduler:spin_once()
	fake.align_status = 1.0
	scheduler:spin_once()
	scheduler:spin_once()
	assert_eq(result, true, "should return success on align success")
	assert_nan(fake.align_direction[#fake.align_direction], "should cancel align request on exit")
end

do
	bb.user.yaw = 0.0
	fake.align_status = 0.3
	fake.align_direction = {}

	local result = nil
	scheduler:append_task(function()
		result = action:blocking_align_chassis(-2.0)
	end)

	scheduler:spin_once()
	scheduler:spin_once()
	assert_near(
		fake.align_direction[#fake.align_direction],
		-2.0 + math.pi / 2,
		"should snap negative align direction to nearest face"
	)
	fake.align_status = -1.0
	scheduler:spin_once()
	scheduler:spin_once()
	assert_eq(result, false, "should return failure on align failure")
	assert_nan(fake.align_direction[#fake.align_direction], "should cancel align request on failure")
end

do
	local cases = {
		{ user_yaw = 0.0, request = math.pi, expected = 0.0 },
		{ user_yaw = 0.0, request = 1.0, expected = 1.0 - math.pi / 2 },
		{ user_yaw = 0.0, request = math.pi / 4, expected = math.pi / 4 },
		{ user_yaw = -0.1, request = -math.pi / 4, expected = -math.pi / 4 },
		{ user_yaw = 2.0, request = 0.1, expected = 0.1 + math.pi / 2 },
	}

	for _, case in ipairs(cases) do
		bb.user.yaw = case.user_yaw
		fake.align_status = 0.3
		fake.align_direction = {}

		local done = false
		scheduler:append_task(function()
			action:blocking_align_chassis(case.request)
			done = true
		end)

		scheduler:spin_once()
		assert_near(
			fake.align_direction[1],
			case.expected,
			"should choose nearest of four chassis faces"
		)

		fake.align_status = 1.0
		scheduler:spin_once()
		scheduler:spin_once()
		scheduler:spin_once()
		assert_eq(done, true, "should finish snapped align")
	end
end

do
	bb.user.yaw = 0 / 0
	fake.align_direction = {}

	local result = action:blocking_align_chassis(1.0)
	assert_eq(result, false, "should fail fast on invalid user yaw")
	assert_eq(#fake.align_direction, 0, "should not request align with invalid user yaw")
	bb.user.yaw = 0.0
end

do
	fake.fold_state = 0
	local done = false
	scheduler:append_task(function()
		action:blocking_set_gimbal_fold(true)
		done = true
	end)

	scheduler:spin_once()
	assert_eq(fake.fold_request, true, "should request folded pose")
	assert_eq(done, false, "should wait for folded pose")

	fake.fold_state = 3
	scheduler:spin_once()
	assert_eq(done, true, "should return after folded pose reached")

	fake.fold_state = 3
	done = false
	scheduler:append_task(function()
		action:blocking_set_gimbal_fold(false)
		done = true
	end)

	scheduler:spin_once()
	assert_eq(fake.fold_request, false, "should request unfolded pose")
	assert_eq(done, false, "should wait for unfolded pose")

	fake.fold_state = 0
	scheduler:spin_once()
	assert_eq(done, true, "should return after unfolded pose reached")
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
