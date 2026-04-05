local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local root = script_dir .. "../.."

package.path = table.concat({
	root .. "/src/lua/?.lua",
	root .. "/src/lua/?/init.lua",
	root .. "/src/lua/?/?.lua",
	package.path,
}, ";")

local function assert_eq(actual, expected, message)
	if actual ~= expected then
		error(string.format("%s: expected %s, got %s", message, tostring(expected), tostring(actual)))
	end
end

local function assert_true(value, message)
	assert_eq(value, true, message)
end

local function assert_false(value, message)
	assert_eq(value, false, message)
end

local Fsm = require("util.fsm")

local State = {
	IDLE = "idle",
	RUN = "run",
	MOVE = "move",
}

local fsm = Fsm:new(State.IDLE)
local trace = {}
local run_count = 0

assert_false(fsm:init_ready(State), "init_ready should fail before registration")

fsm:use({
	state = State.IDLE,
	enter = function()
		trace[#trace + 1] = "idle:enter"
	end,
	event = function(handle)
		trace[#trace + 1] = "idle:event"
		assert_eq(handle:last_state(), nil, "idle last_state before first transition")
		handle:set_next(State.RUN)
	end,
})

fsm:use({
	state = State.RUN,
	enter = function()
		trace[#trace + 1] = "run:enter"
	end,
	event = function(handle)
		run_count = run_count + 1
		trace[#trace + 1] = "run:event"
		if run_count == 1 then
			assert_eq(handle:last_state(), State.IDLE, "run last_state after transition")
		else
			assert_eq(handle:last_state(), nil, "run last_state after start_on")
		end
		handle:set_next(State.MOVE)
	end,
})

fsm:use({
	state = State.MOVE,
	enter = function()
		trace[#trace + 1] = "move:enter"
	end,
	event = function(handle)
		trace[#trace + 1] = "move:event"
		assert_eq(handle:last_state(), State.RUN, "move last_state after transition")
	end,
})

assert_true(fsm:init_ready(State), "init_ready should pass after all states are registered")

fsm:spin_once()
fsm:spin_once()
fsm:spin_once()

fsm:start_on(State.RUN)
fsm:spin_once()
fsm:spin_once()

local expected_trace = {
	"idle:enter",
	"idle:event",
	"run:enter",
	"run:event",
	"move:enter",
	"move:event",
	"run:enter",
	"run:event",
	"move:enter",
	"move:event",
}

assert_eq(#trace, #expected_trace, "trace length")
for i = 1, #expected_trace do
	assert_eq(trace[i], expected_trace[i], string.format("trace[%d]", i))
end

print("fsm.lua: ok")
