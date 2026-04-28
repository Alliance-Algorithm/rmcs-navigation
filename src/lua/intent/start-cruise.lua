local fsm = require("util.fsm")
local Region = require("region")
local cross_fluctuant_road = require("task.cross-fluctuant.cross-fluctuant-road")
local navigate_to_fluctuant_begin = require("task.cross-fluctuant.navigate-to-fluctuant-begin")

local StartCruiseIntent = {}
StartCruiseIntent.__index = StartCruiseIntent

local State = {
	to_fluctuant_begin = "to_fluctuant_begin",
	crossing_fluctuant = "crossing_fluctuant",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

local function initial_state(ctx)
	if Region.is_after_fluctuant(ctx.region()) then
		return State.done
	end
	if Region.is_on_fluctuant(ctx.region()) then
		return State.crossing_fluctuant
	end
	return State.to_fluctuant_begin
end

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		phase = "none",
		status = "running",
		machine = nil,
	}, StartCruiseIntent)
end

function StartCruiseIntent:phase_name()
	return self.phase
end

function StartCruiseIntent:create_machine(ctx)
	local machine = fsm:new(initial_state(ctx))

	machine:use({
		state = State.to_fluctuant_begin,
		enter = function()
			self.phase = State.to_fluctuant_begin
			ctx.run_job("navigate_to_fluctuant_begin", function()
				return navigate_to_fluctuant_begin(self.ours_zone, true)
			end)
		end,
		event = function(handle)
			if Region.is_after_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.done)
				return
			end

			if Region.is_on_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.crossing_fluctuant)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.crossing_fluctuant)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.crossing_fluctuant,
		enter = function()
			self.phase = State.crossing_fluctuant
			ctx.run_job("cross_fluctuant", function()
				return cross_fluctuant_road(self.ours_zone, true)
			end)
		end,
		event = function(handle)
			if Region.is_after_fluctuant(ctx.region()) then
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

	assert(machine:init_ready(State), "start-cruise intent fsm init_ready failed")
	return machine
end

function StartCruiseIntent:spin(ctx)
	assert(type(ctx) == "table", "ctx should be a table")
	if self.machine == nil then
		self.machine = self:create_machine(ctx)
	end
	if self.status == "running" then
		self.machine:spin_once()
	end
	return self.status
end

return M
