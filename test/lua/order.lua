local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq

package.loaded["util.clock"] = nil
package.loaded["util.order"] = nil
local clock = require("util.clock")
local order = require("util.order")

local initial_value = "MIDDLE"
local initial_triggered = 0
local initial_event = order.new(function()
	return initial_value
end, 0.5)

initial_event:on({ "MIDDLE", "UP", "MIDDLE" }, function()
	initial_triggered = initial_triggered + 1
end)

clock:reset(0)
clock:update(1.0)
initial_value = "UP"
initial_event:spin()
clock:update(1.2)
initial_value = "MIDDLE"
initial_event:spin()
assert_eq(initial_triggered, 1, "initial first state should be primed without reset")

local value = "DOWN"
local triggered = 0
local event = order.new(function()
	return value
end, 0.5)

event:on({ "MIDDLE", "UP", "MIDDLE" }, function()
	triggered = triggered + 1
end)

clock:reset(0)
event:reset()

clock:update(0.1)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 0, "first step should not trigger")

clock:update(10.0)
value = "UP"
event:spin()
assert_eq(triggered, 0, "second step should not trigger")

clock:update(10.3)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 1, "complete sequence should trigger once")

clock:update(10.5)
event:spin()
assert_eq(triggered, 1, "holding value should not retrigger")

clock:update(10.7)
value = "UP"
event:spin()
clock:update(10.9)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 2, "resting on first state should allow repeated trigger")

clock:update(11.1)
value = "UP"
event:spin()
clock:update(11.3)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 3, "resting on first state should allow another repeat")

clock:update(11.5)
value = "UP"
event:spin()
clock:update(11.7)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 4, "overlapping sequence should trigger again")

clock:update(12.0)
value = "DOWN"
event:spin()
clock:update(12.1)
value = "UP"
event:spin()
clock:update(12.2)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 4, "sequence should not continue from invalid start")

clock:update(13.0)
value = "MIDDLE"
event:spin()
clock:update(13.2)
value = "UP"
event:spin()
clock:update(13.8)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 4, "timeout should reset sequence")

clock:update(14.0)
value = "DOWN"
event:spin()
clock:update(14.1)
value = "MIDDLE"
event:spin()
clock:update(14.3)
value = "DOWN"
event:spin()
clock:update(14.4)
value = "MIDDLE"
event:spin()
clock:update(14.6)
value = "UP"
event:spin()
clock:update(14.8)
value = "MIDDLE"
event:spin()
assert_eq(triggered, 5, "new start after interruption should work")

do
	local short_count = 0
	local long_count = 0
	local v = "DOWN"
	local multi = order.new(function()
		return v
	end, 0.5)

	multi:on({ "MIDDLE", "UP", "MIDDLE" }, function()
		short_count = short_count + 1
	end)
	multi:on({ "MIDDLE", "UP", "MIDDLE", "UP", "MIDDLE" }, function()
		long_count = long_count + 1
	end)

	clock:reset(0)
	multi:reset()

	clock:update(1.0)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.2)
	v = "UP"
	multi:spin()
	clock:update(1.4)
	v = "MIDDLE"
	multi:spin()
	assert_eq(short_count, 0, "short prefix should wait for longer contender")
	assert_eq(long_count, 0, "long sequence not finished yet")

	clock:update(2.0)
	multi:spin()
	assert_eq(short_count, 1, "short should fire after longer times out")
	assert_eq(long_count, 0, "long should not fire after timeout")
end

do
	local short_count = 0
	local long_count = 0
	local v = "DOWN"
	local multi = order.new(function()
		return v
	end, 0.5)

	multi:on({ "MIDDLE", "UP", "MIDDLE" }, function()
		short_count = short_count + 1
	end)
	multi:on({ "MIDDLE", "UP", "MIDDLE", "UP", "MIDDLE" }, function()
		long_count = long_count + 1
	end)

	clock:reset(0)
	multi:reset()

	clock:update(1.0)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.2)
	v = "UP"
	multi:spin()
	clock:update(1.4)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.6)
	v = "UP"
	multi:spin()
	clock:update(1.8)
	v = "MIDDLE"
	multi:spin()
	assert_eq(short_count, 0, "short should be suppressed when long completes")
	assert_eq(long_count, 1, "only long sequence should fire")
end

do
	local short_count = 0
	local long_count = 0
	local v = "DOWN"
	local multi = order.new(function()
		return v
	end, 0.5)

	multi:on({ "MIDDLE", "UP", "MIDDLE" }, function()
		short_count = short_count + 1
	end)
	multi:on({ "MIDDLE", "UP", "MIDDLE", "UP", "MIDDLE" }, function()
		long_count = long_count + 1
	end)

	clock:reset(0)
	multi:reset()

	clock:update(1.0)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.2)
	v = "UP"
	multi:spin()
	clock:update(1.4)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.6)
	v = "DOWN"
	multi:spin()
	assert_eq(short_count, 1, "short should fire when longer diverges")
	assert_eq(long_count, 0, "long should not fire after diverge")
end

do
	local a_count = 0
	local b_count = 0
	local v = "DOWN"
	local multi = order.new(function()
		return v
	end, 0.5)

	multi:on({ "MIDDLE", "UP", "MIDDLE" }, function()
		a_count = a_count + 1
	end)
	multi:on({ "DOWN", "UP", "DOWN" }, function()
		b_count = b_count + 1
	end)

	clock:reset(0)
	multi:reset()

	clock:update(1.0)
	v = "MIDDLE"
	multi:spin()
	clock:update(1.2)
	v = "UP"
	multi:spin()
	clock:update(1.4)
	v = "MIDDLE"
	multi:spin()
	assert_eq(a_count, 1, "unrelated sequence A should fire immediately")
	assert_eq(b_count, 0, "unrelated sequence B should not fire")

	clock:update(2.0)
	v = "DOWN"
	multi:spin()
	clock:update(2.2)
	v = "UP"
	multi:spin()
	clock:update(2.4)
	v = "DOWN"
	multi:spin()
	assert_eq(a_count, 1, "unrelated A should stay")
	assert_eq(b_count, 1, "unrelated sequence B should fire independently")
end

print("order.lua: ok")
