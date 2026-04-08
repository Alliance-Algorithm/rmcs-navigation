local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local scheduler = require("util.scheduler")
local clock = require("util.clock")

local assert_eq = test_util.assert_eq
local assert_false = test_util.assert_false
local assert_true = test_util.assert_true
local assert_table_eq = test_util.assert_table_eq

local function step(ctx, now)
	clock:update(now)
	ctx:spin_once()
end

local function task_of(fn, trace)
	return function()
		fn(trace)
	end
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "start"
		scheduler.request:yield()
		items[#items + 1] = "after_yield"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "start" }, "yield first spin")

	step(ctx, 0)
	assert_table_eq(trace, { "start", "after_yield" }, "yield second spin")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_sleep"
		scheduler.request:sleep(1.0)
		items[#items + 1] = "after_sleep"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "before_sleep" }, "sleep initial spin")

	step(ctx, 0.5)
	assert_table_eq(trace, { "before_sleep" }, "sleep should block before deadline")

	step(ctx, 1.0)
	assert_table_eq(trace, { "before_sleep", "after_sleep" }, "sleep should resume at deadline")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_zero_sleep"
		scheduler.request:sleep(0.0)
		items[#items + 1] = "after_zero_sleep"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "before_zero_sleep" }, "zero sleep should still suspend current spin")

	step(ctx, 0)
	assert_table_eq(trace, { "before_zero_sleep", "after_zero_sleep" }, "zero sleep should resume next spin")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}
	local ready = false

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_wait"
		local is_timeout = scheduler.request:wait_until {
			monitor = function()
				return ready
			end,
			timeout = 5.0,
		}
		items[#items + 1] = is_timeout and "timeout" or "ready"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "before_wait" }, "wait_until initial spin")

	step(ctx, 1.0)
	assert_table_eq(trace, { "before_wait" }, "wait_until should block before monitor ready")

	ready = true
	step(ctx, 2.0)
	assert_table_eq(trace, { "before_wait", "ready" }, "wait_until should resume when monitor succeeds")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}
	local monitor_calls = 0

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_immediate_wait"
		local is_timeout = scheduler.request:wait_until {
			monitor = function()
				monitor_calls = monitor_calls + 1
				return true
			end,
			timeout = 5.0,
		}
		items[#items + 1] = is_timeout and "timeout" or "ready"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "before_immediate_wait" }, "wait_until immediate success should still suspend current spin")
	assert_eq(monitor_calls, 0, "wait_until monitor should not run until next spin")

	step(ctx, 0)
	assert_table_eq(trace, { "before_immediate_wait", "ready" }, "wait_until should resume on next spin when monitor already succeeds")
	assert_eq(monitor_calls, 1, "wait_until immediate success monitor calls")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_wait"
		local is_timeout = scheduler.request:wait_until {
			monitor = function()
				return false
			end,
			timeout = 1.0,
		}
		items[#items + 1] = is_timeout and "timeout" or "ready"
	end, trace))

	step(ctx, 0)
	step(ctx, 0.5)
	assert_table_eq(trace, { "before_wait" }, "wait_until timeout should not trigger early")

	step(ctx, 1.1)
	assert_table_eq(trace, { "before_wait", "timeout" }, "wait_until should report timeout")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "a0"
		scheduler.request:sleep(1.0)
		items[#items + 1] = "a1"
	end, trace))

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "b0"
		scheduler.request:yield()
		items[#items + 1] = "b1"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "a0", "b0" }, "multiple tasks first spin order")

	step(ctx, 0)
	assert_table_eq(trace, { "a0", "b0", "b1" }, "yielding task should resume before sleeping task")

	step(ctx, 1.0)
	assert_table_eq(trace, { "a0", "b0", "b1", "a1" }, "sleeping task should resume when ready")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "step0"
		scheduler.request:yield()
		items[#items + 1] = "step1"
		scheduler.request:yield()
		items[#items + 1] = "step2"
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "step0" }, "multiple yields first spin")

	step(ctx, 0)
	assert_table_eq(trace, { "step0", "step1" }, "multiple yields second spin")

	step(ctx, 0)
	assert_table_eq(trace, { "step0", "step1", "step2" }, "multiple yields third spin")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	ctx:append_task(task_of(function(items)
		items[#items + 1] = "done"
	end, trace))

	local ok = pcall(function()
		step(ctx, 0)
	end)
	assert_eq(ok, true, "completed task should not crash scheduler")
	assert_table_eq(trace, { "done" }, "completed task trace")

	ok = pcall(function()
		step(ctx, 1.0)
	end)
	assert_eq(ok, true, "completed task should be removable on next spin")
end

do
	clock:reset(0)
	local ctx = scheduler.new()
	local trace = {}

	local handle = ctx:append_task(task_of(function(items)
		items[#items + 1] = "before_loop"
		while true do
			scheduler.request:yield()
			items[#items + 1] = "loop"
		end
	end, trace))

	step(ctx, 0)
	assert_table_eq(trace, { "before_loop" }, "cancel task first spin")

	step(ctx, 0)
	assert_table_eq(trace, { "before_loop", "loop" }, "cancel task second spin")

	handle.cancel()
	step(ctx, 0)
	assert_table_eq(trace, { "before_loop", "loop" }, "cancelled task should not continue")
end

do
	clock:reset(0)
	local ctx = scheduler.new()

	ctx:append_task(function()
		scheduler.request:yield()
		error("task panic")
	end)

	local ok = pcall(function()
		step(ctx, 0)
	end)
	assert_true(ok, "task error should not trigger before panic point")

	local ok_error, err = pcall(function()
		step(ctx, 0)
	end)
	assert_false(ok_error, "task error should be raised to spin_once caller")
	assert_true(type(err) == "string" and string.find(err, "task panic", 1, true) ~= nil,
		"task error message should include coroutine panic")
end

clock:reset()
assert_false(clock:is_ready(), "scheduler tests should restore clock state")

print("scheduler.lua: ok")
