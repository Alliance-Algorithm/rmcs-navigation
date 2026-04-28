local blackboard = require("blackboard").singleton()
local action = require("action")
local fsm = require("util.fsm")
local Region = require("region")
local go_down_onestep = require("task.one-step.go-down-onestep")
local cross_fluctuant_road = require("task.cross-fluctuant.cross-fluctuant-road")
local navigate_to_point = require("task.navigate-to-point")

local EscapeToHomeIntent = {}
EscapeToHomeIntent.__index = EscapeToHomeIntent

local State = {
	descend_onestep = "descend_onestep",
	cross_fluctuant = "cross_fluctuant",
	to_resupply = "to_resupply",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

local function initial_state(route)
	if route == "onestep" then
		return State.descend_onestep
	end
	if route == "fluctuant_road" then
		return State.cross_fluctuant
	end
	return State.to_resupply
end

function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.route) == "string", "args.route should be a string")

	return setmetatable({
		route = args.route,
		phase = "none",
		status = "running",
		machine = nil,
	}, EscapeToHomeIntent)
end

function EscapeToHomeIntent:phase_name()
	return self.phase
end

function EscapeToHomeIntent:create_machine(ctx)
	local machine = fsm:new(initial_state(self.route))
	local resupply_zone = blackboard.rule.resupply_zone.ours

	machine:use({
		state = State.descend_onestep,
		enter = function()
			self.phase = State.descend_onestep
			action:info("escape-to-home: 开始走一级台阶回家")
			ctx.run_job("escape_descend_onestep", function()
				return go_down_onestep(true)
			end)
		end,
		event = function(handle)
			if not Region.is_after_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.to_resupply)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.to_resupply)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.cross_fluctuant,
		enter = function()
			self.phase = State.cross_fluctuant
			action:info("escape-to-home: 开始走起伏路回家")
			ctx.run_job("escape_cross_fluctuant", function()
				return cross_fluctuant_road(true, false)
			end)
		end,
		event = function(handle)
			if Region.is_before_fluctuant(ctx.region()) then
				ctx.cancel_job()
				handle:set_next(State.to_resupply)
				return
			end

			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.to_resupply)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.to_resupply,
		enter = function()
			self.phase = State.to_resupply
			action:update_chassis_mode("SPIN")
			ctx.run_job("escape_to_resupply", function()
				return navigate_to_point(resupply_zone, {
					tolerance = 0.15,
					timeout = 10,
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
			action:info("escape-to-home: 已抵达补给点")
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

	assert(machine:init_ready(State), "escape-to-home intent fsm init_ready failed")
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
