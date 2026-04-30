local action = require("action")
local fsm = require("util.fsm")
local cross_road_zone = require("task.cross-road.cross-road-zone")

local CrossRoadIntent = {}
CrossRoadIntent.__index = CrossRoadIntent

local State = {
	crossing = "crossing",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")
	if args.forward_center == nil then
		args.forward_center = true
	end
	assert(type(args.forward_center) == "boolean", "args.forward_center should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		forward_center = args.forward_center,
		phase = "none",
		status = "running",
		machine = nil,
	}, CrossRoadIntent)
end

function CrossRoadIntent:phase_name()
	return self.phase
end

function CrossRoadIntent:create_machine(ctx)
	local machine = fsm:new(State.crossing)

	machine:use({
		state = State.crossing,
		enter = function()
			self.phase = State.crossing
			ctx.run_job("train_cross_road", function()
				action:info("train/cross-road: 开始通过公路区")
				return cross_road_zone(self.ours_zone, self.forward_center)
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
			action:info("train/cross-road: 已通过公路区")
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

	assert(machine:init_ready(State), "train cross-road intent fsm init_ready failed")
	return machine
end

function CrossRoadIntent:spin(ctx)
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
