local action = require("action")
local cruise_in_central_highlands = require("task.cruise-in-central-highland.cruise-in-central-highlands")
local ReturnStage = require("util.return-stage")

local KeepCruiseIntent = {}
KeepCruiseIntent.__index = KeepCruiseIntent

local M = {}

--- @param args { ours_zone: boolean, switch_interval: number }
--- @return table
function M.new(args)
	assert(type(args) == "table", "args should be a table")
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")
	assert(type(args.switch_interval) == "number", "args.switch_interval should be a number")
	assert(args.switch_interval > 0, "args.switch_interval should be positive")

		return setmetatable({
			ours_zone = args.ours_zone,
			switch_interval = args.switch_interval,
			_return_stage = ReturnStage.after_fluctuant,
		}, KeepCruiseIntent)
end

--- @return "after_fluctuant"
function KeepCruiseIntent:return_stage()
	return self._return_stage
end

--- @param run_job fun(name: string, fn: function)
function KeepCruiseIntent:run(run_job)
	assert(type(run_job) == "function", "run_job should be a function")

	run_job("keep_cruise", function()
		action:info("keep-cruise: 进入中央高地持续巡航")
		local ok = cruise_in_central_highlands(self.ours_zone, self.switch_interval)
		if not ok then
			action:warn("keep-cruise: 中央高地巡航导航失败")
			return false
		end

		return true
	end)
end

return M
