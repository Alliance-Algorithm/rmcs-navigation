local forward_press_in_one_step = require("task.forward-press.forward-press-in-one-step")
local forward_press_in_two_step = require("task.forward-press.forward-press-in-two-step")
local ReturnStage = require("util.return-stage")

local ForwardPressIntent = {}
ForwardPressIntent.__index = ForwardPressIntent

local Phase = {
	one_step = "one_step",
	two_step = "two_step",
}

local function unknown_phase_error(phase)
	error("unknown forward-press intent phase: " .. tostring(phase))
end

local M = {
	Phase = Phase,
}

--- @param args { mode: "one_step"|"two_step", switch_interval: number }
--- @return table
function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.mode) == "string", "args.mode should be a string")
	assert(type(args.switch_interval) == "number", "args.switch_interval should be a number")
	assert(args.switch_interval > 0, "args.switch_interval should be positive")

		return setmetatable({
			phase = args.mode,
			switch_interval = args.switch_interval,
			_return_stage = ReturnStage.after_fluctuant,
		}, ForwardPressIntent)
end

--- @return string
function ForwardPressIntent:phase_name()
	return self.phase
end

--- @return "after_fluctuant"
function ForwardPressIntent:return_stage()
	return self._return_stage
end

--- @param run_job fun(name: string, fn: function)
function ForwardPressIntent:run(run_job)
	assert(type(run_job) == "function", "run_job should be a function")

	if self.phase == Phase.one_step then
		run_job("forward_press_in_one_step", function()
			return forward_press_in_one_step()
		end)
		return
	end

	if self.phase == Phase.two_step then
		run_job("forward_press_in_two_step", function()
			return forward_press_in_two_step(self.switch_interval)
		end)
		return
	end

	unknown_phase_error(self.phase)
end

return M
