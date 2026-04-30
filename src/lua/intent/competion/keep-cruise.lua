local action = require("action")
local fsm = require("util.fsm")
local cruise_in_central_highlands = require("task.cruise-in-central-highland.cruise-in-central-highlands")

local KeepCruiseIntent = {}
KeepCruiseIntent.__index = KeepCruiseIntent

local State = {
	cruising = "cruising",
	failed = "failed",
}

local M = {
	State = State,
}

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")
	assert(type(args.switch_interval) == "number", "args.switch_interval should be a number")
	assert(args.switch_interval > 0, "args.switch_interval should be positive")

	return setmetatable({
		ours_zone = args.ours_zone,
		switch_interval = args.switch_interval,
		phase = "none",
		status = "running",
		machine = nil,
	}, KeepCruiseIntent)
end

function KeepCruiseIntent:phase_name()
	return self.phase
end

function KeepCruiseIntent:create_machine(ctx)
	local machine = fsm:new(State.cruising)

	machine:use({
		state = State.cruising,
		enter = function()
			self.phase = State.cruising
			ctx.run_job("keep_cruise", function()
				action:info("keep-cruise: 进入中央高地持续巡航")
				return cruise_in_central_highlands(self.ours_zone, self.switch_interval)
			end)
		end,
		event = function(handle)
			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				ctx.run_job("keep_cruise", function()
					action:info("keep-cruise: 巡航任务提前结束，重新进入巡航")
					return cruise_in_central_highlands(self.ours_zone, self.switch_interval)
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

	assert(machine:init_ready(State), "keep-cruise intent fsm init_ready failed")
	return machine
end

function KeepCruiseIntent:spin(ctx)
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
