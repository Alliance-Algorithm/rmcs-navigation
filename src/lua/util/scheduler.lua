local clock = require("util.clock")

--- @class Scheduler
--- @field details table
local scheduler = {}
scheduler.__index = scheduler

local function new_scheduler()
	return setmetatable({
		details = {
			tasks = {},
		},
	}, scheduler)
end

function scheduler:append_task()
	local task = {
		thread = nil,
		resume_ready = function()
			return true
		end,
		remove_ready = function()
			return false
		end,
	}
	table.insert(self.details.tasks, task)
end

function scheduler:spin_once()
	--
end

--- @class SchedulerRequest
local scheduler_request = {}
scheduler_request.__index = scheduler_request

function scheduler_request:yield()
	coroutine.yield {
		resume_ready = function()
			return true
		end,
	}
end

function scheduler_request:sleep(seconds)
	assert(type(seconds) == "number" and seconds >= 0)

	local deadline = clock:now() + seconds
	coroutine.yield {
		resume_ready = function()
			return clock:now() >= deadline
		end,
	}
end

--- @param args { monitor: function, timeout?: number }
--- @return boolean is_timeout
function scheduler_request:wait_until(args)
	assert(type(args.monitor) == "function")

	local deadline = clock:now() + (args.timeout or math.huge)
	coroutine.yield {
		resume_ready = function()
			local success = args.monitor()
			local timeout = clock:now() > deadline
			return success or timeout
		end,
	}
	return clock:now() > deadline
end

return {
	new = new_scheduler,
	request = scheduler_request,
}
