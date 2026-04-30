local clock = require("util.clock")
local fsm = require("util.fsm")
local forward_press_in_one_step = require("task.forward-press.forward-press-in-one-step")
local forward_press_in_two_step = require("task.forward-press.forward-press-in-two-step")

local ForwardPressIntent = {}
ForwardPressIntent.__index = ForwardPressIntent

local State = {
	one_step = "one_step",
	two_step = "two_step",
	hold = "hold",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

local function initial_state(mode)
	if mode == "two_step" then
		return State.two_step
	end
	return State.one_step
end

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.mode) == "string", "args.mode should be a string")
	assert(type(args.switch_interval) == "number", "args.switch_interval should be a number")
	assert(args.switch_interval > 0, "args.switch_interval should be positive")

	return setmetatable({
		mode = args.mode,
		switch_interval = args.switch_interval,
		duration = args.duration or 30.0,
		phase = "none",
		status = "running",
		started_at = nil,
		machine = nil,
	}, ForwardPressIntent)
end

function ForwardPressIntent:phase_name()
	return self.phase
end

function ForwardPressIntent:elapsed()
	return clock:now() - self.started_at
end

function ForwardPressIntent:create_machine(ctx)
	local machine = fsm:new(initial_state(self.mode))

	machine:use({
		state = State.one_step,
		enter = function()
			self.phase = State.one_step
			ctx.run_job("forward_press_in_one_step", function()
				return forward_press_in_one_step()
			end)
		end,
		event = function(handle)
			if self:elapsed() >= self.duration then
				ctx.cancel_job()
				handle:set_next(State.done)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.hold)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.two_step,
		enter = function()
			self.phase = State.two_step
			ctx.run_job("forward_press_in_two_step", function()
				return forward_press_in_two_step(self.switch_interval)
			end)
		end,
		event = function(handle)
			if self:elapsed() >= self.duration then
				ctx.cancel_job()
				handle:set_next(State.done)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.done)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.hold,
		enter = function()
			self.phase = State.hold
		end,
		event = function(handle)
			if self:elapsed() >= self.duration then
				handle:set_next(State.done)
			end
		end,
	})

	machine:use({
		state = State.done,
		enter = function()
			self.phase = State.done
			self.status = "success"
		end,
		event = function() end,
	})

	machine:use({
		state = State.failed,
		enter = function()
			self.phase = State.failed
			self.status = "failed"
		end,
		event = function() end,
	})

	assert(machine:init_ready(State), "forward-press intent fsm init_ready failed")
	return machine
end

function ForwardPressIntent:spin(ctx)
	assert(type(ctx) == "table", "ctx should be a table")
	if self.started_at == nil then
		self.started_at = clock:now()
	end
	if self.machine == nil then
		self.machine = self:create_machine(ctx)
	end
	if self.status == "running" then
		self.machine:spin_once()
	end
	return self.status
end

return M
