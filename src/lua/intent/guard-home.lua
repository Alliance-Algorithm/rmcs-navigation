local go_down_onestep = require("task.one-step.go-down-onestep")
local cruise_in_front_of_base = require("task.guard-home.cruise-in-front-of-base")
local occupy_fortress = require("task.guard-home.occupy-fortress")
local ReturnStage = require("util.return-stage")

local GuardHomeIntent = {}
GuardHomeIntent.__index = GuardHomeIntent

local Phase = {
	descend_onestep = "descend_onestep",
	occupy_fortress = "occupy_fortress",
	cruise_in_front_of_base = "cruise_in_front_of_base",
}

local function unknown_phase_error(phase)
	error("unknown guard-home intent phase: " .. tostring(phase))
end

local M = {
	Phase = Phase,
}

--- @param args { phase: "occupy_fortress"|"cruise_in_front_of_base", return_stage: string }
--- @return table
function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.phase) == "string", "args.phase should be a string")
	assert(type(args.return_stage) == "string", "args.return_stage should be a string")

	local current_phase = args.phase
	if args.return_stage == ReturnStage.after_fluctuant then
		current_phase = Phase.descend_onestep
	end

	return setmetatable({
		phase = current_phase,
		target_phase = args.phase,
		_return_stage = args.return_stage,
	}, GuardHomeIntent)
end

--- @return string
function GuardHomeIntent:phase_name()
	return self.target_phase
end

--- @return "before_fluctuant"|"after_fluctuant"
function GuardHomeIntent:return_stage()
	return self._return_stage
end

function GuardHomeIntent:on_job_succeeded()
	if self.phase == Phase.descend_onestep then
		self._return_stage = ReturnStage.before_fluctuant
	end
end

function GuardHomeIntent:advance()
	if self.phase == Phase.descend_onestep then
		self.phase = self.target_phase
		return true
	end

	return false
end

--- @param run_job fun(name: string, fn: function)
function GuardHomeIntent:run(run_job)
	assert(type(run_job) == "function", "run_job should be a function")

	if self.phase == Phase.descend_onestep then
		run_job("guard_home_descend_onestep", function()
			return go_down_onestep(true)
		end)
		return
	end

	if self.phase == Phase.occupy_fortress then
		run_job("occupy_fortress", function()
			return occupy_fortress()
		end)
		return
	end

	if self.phase == Phase.cruise_in_front_of_base then
		run_job("cruise_in_front_of_base", function()
			return cruise_in_front_of_base(true)
		end)
		return
	end

	unknown_phase_error(self.phase)
end

return M
