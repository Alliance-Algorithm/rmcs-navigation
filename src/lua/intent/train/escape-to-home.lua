local blackboard = require("blackboard").singleton()
local action = require("action")
local fsm = require("util.fsm")
local navigate_to_point = require("task.navigate-to-point")

local EscapeToHomeIntent = {}
EscapeToHomeIntent.__index = EscapeToHomeIntent

local State = {
	to_resupply = "to_resupply",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

function M.new(args)
	args = args or {}
	assert(type(args) == "table", "args should be a table")
	if args.ours_zone == nil then
		args.ours_zone = true
	end
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		tolerance = args.tolerance or 0.15,
		timeout = args.timeout or 10.0,
		phase = "none",
		status = "running",
		machine = nil,
	}, EscapeToHomeIntent)
end

function EscapeToHomeIntent:phase_name()
	return self.phase
end

function EscapeToHomeIntent:create_machine(ctx)
	local machine = fsm:new(State.to_resupply)

	machine:use({
		state = State.to_resupply,
		enter = function()
			self.phase = State.to_resupply
			action:update_chassis_mode("SPIN")
			ctx.run_job("train_escape_to_resupply", function()
				local resupply_zone = self.ours_zone
					and blackboard.rule.resupply_zone.ours
					or blackboard.rule.resupply_zone.them
				action:info("train/escape-to-home: 导航到补给点")
				return navigate_to_point(resupply_zone, {
					tolerance = self.tolerance,
					timeout = self.timeout,
				})
			end)
		end,
		event = function(handle)
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
			action:info("train/escape-to-home: 已抵达补给点")
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

	assert(machine:init_ready(State), "train escape-to-home intent fsm init_ready failed")
	return machine
end

function EscapeToHomeIntent:spin(ctx)
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
