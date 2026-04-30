local fsm = require("util.fsm")
local Region = require("region")
local go_down_onestep = require("task.one-step.go-down-onestep")
local cruise_in_front_of_base = require("task.guard-home.cruise-in-front-of-base")
local occupy_fortress = require("task.guard-home.occupy-fortress")

local GuardHomeIntent = {}
GuardHomeIntent.__index = GuardHomeIntent

local State = {
	descend_onestep = "descend_onestep",
	occupy_fortress = "occupy_fortress",
	cruise_in_front_of_base = "cruise_in_front_of_base",
	failed = "failed",
}

local M = {
	State = State,
}

local function target_state(ctx)
	local target = ctx.guard_home_target()
	if target == State.cruise_in_front_of_base then
		return State.cruise_in_front_of_base
	end
	return State.occupy_fortress
end

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		phase = "none",
		status = "running",
		machine = nil,
	}, GuardHomeIntent)
end

function GuardHomeIntent:phase_name()
	return self.phase
end

function GuardHomeIntent:create_machine(ctx)
	local start_state = Region.is_after_fluctuant(ctx.region()) and State.descend_onestep
		or target_state(ctx)
	local machine = fsm:new(start_state)

	machine:use({
		state = State.descend_onestep,
		enter = function()
			self.phase = State.descend_onestep
			ctx.run_job("guard_home_descend_onestep", function()
				return go_down_onestep(self.ours_zone)
			end)
		end,
		event = function(handle)
			if not Region.is_after_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(target_state(ctx))
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(target_state(ctx))
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.occupy_fortress,
		enter = function()
			self.phase = State.occupy_fortress
			ctx.run_job("occupy_fortress", function()
				return occupy_fortress()
			end)
		end,
		event = function(handle)
			if Region.is_after_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.descend_onestep)
				return
			end

			if target_state(ctx) == State.cruise_in_front_of_base then
				if not ctx.job_state().done then
					ctx.cancel_job()
				end
				handle:set_next(State.cruise_in_front_of_base)
				return
			end

			local job = ctx.job_state()
			if job.done and not job.success then
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.cruise_in_front_of_base,
		enter = function()
			self.phase = State.cruise_in_front_of_base
			ctx.run_job("cruise_in_front_of_base", function()
				return cruise_in_front_of_base(self.ours_zone)
			end)
		end,
		event = function(handle)
			if Region.is_after_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.descend_onestep)
				return
			end

			if target_state(ctx) == State.occupy_fortress then
				ctx.cancel_job()
				handle:set_next(State.occupy_fortress)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				ctx.run_job("cruise_in_front_of_base", function()
					return cruise_in_front_of_base(self.ours_zone)
				end)
				return
			end

			handle:set_next(State.failed)
		end,
	})

	machine:use({
		state = State.failed,
		enter = function()
			self.phase = State.failed
			self.status = "failed"
		end,
		event = function() end,
	})

	assert(machine:init_ready(State), "guard-home intent fsm init_ready failed")
	return machine
end

function GuardHomeIntent:spin(ctx)
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
