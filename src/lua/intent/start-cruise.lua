local cross_fluctuant_road = require("task.cross-fluctuant.cross-fluctuant-road")
local navigate_to_fluctuant_begin = require("task.cross-fluctuant.navigate-to-fluctuant-begin")
local ReturnStage = require("util.return-stage")

local StartCruiseIntent = {}
StartCruiseIntent.__index = StartCruiseIntent

local Phase = {
	to_fluctuant_begin = "to_fluctuant_begin",
	crossing_fluctuant = "crossing_fluctuant",
}

local function unknown_phase_error(phase)
	error("unknown start-cruise intent phase: " .. tostring(phase))
end

local M = {
	Phase = Phase,
}

--- @param args { ours_zone: boolean }
--- @return table
function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		phase = Phase.to_fluctuant_begin,
		_return_stage = ReturnStage.before_fluctuant,
	}, StartCruiseIntent)
end

--- @return string
function StartCruiseIntent:phase_name()
	return self.phase
end

--- @return "before_fluctuant"|"on_fluctuant"|"after_fluctuant"
function StartCruiseIntent:return_stage()
	return self._return_stage
end

--- @param run_job fun(name: string, fn: function)
function StartCruiseIntent:run(run_job)
	assert(type(run_job) == "function", "run_job should be a function")

	if self.phase == Phase.to_fluctuant_begin then
		run_job("navigate_to_fluctuant_begin", function()
			return navigate_to_fluctuant_begin(self.ours_zone, true)
		end)
		return
	end

	if self.phase == Phase.crossing_fluctuant then
		run_job("cross_fluctuant", function()
			return cross_fluctuant_road(self.ours_zone, true)
		end)
		return
	end

	unknown_phase_error(self.phase)
end

--- @return boolean has_next_phase
function StartCruiseIntent:advance()
	if self.phase == Phase.to_fluctuant_begin then
		self.phase = Phase.crossing_fluctuant
		self._return_stage = ReturnStage.on_fluctuant
		return true
	end

	if self.phase == Phase.crossing_fluctuant then
		return false
	end

	unknown_phase_error(self.phase)
end

function StartCruiseIntent:on_job_succeeded()
	if self.phase == Phase.crossing_fluctuant then
		self._return_stage = ReturnStage.after_fluctuant
	end
end

return M
