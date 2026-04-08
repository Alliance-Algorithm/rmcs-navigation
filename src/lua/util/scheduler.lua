local clock = require("util.clock")

--- @class SchedulerTask
--- @field thread thread
--- @field resume_request function
--- @field cancel_request function

--- @class SchedulerDetails
--- @field tasks SchedulerTask[]

--- @class SchedulerRequestArgs
--- @field resume_request function

--- @class Scheduler
--- @field details SchedulerDetails
local scheduler = {}
scheduler.__index = scheduler

--- @param fn function
--- @return { cancel: function } task
function scheduler:append_task(fn)
	local cancel_status = false

	--- @type SchedulerTask
	local task = {
		thread = coroutine.create(fn),
		resume_request = function()
			return true
		end,
		cancel_request = function()
			return cancel_status
		end,
	}
	table.insert(self.details.tasks, task)

	return {
		cancel = function()
			cancel_status = true
		end,
	}
end

function scheduler:spin_once()
	--- @type SchedulerDetails
	local details = self.details

	--- @type SchedulerTask[]
	local saved_tasks = {}
	for _, task in ipairs(details.tasks) do
		-- 应该被移除的任务
		local status = coroutine.status(task.thread)
		local cancel = task.cancel_request()
		if status == "dead" or cancel then
			goto continue
		end

		if task.resume_request() then
			local result, request = coroutine.resume(task.thread)
			if result ~= true then
				error(debug.traceback(task.thread, tostring(request)), 0)
			end

			--- @cast request SchedulerRequestArgs
			if request ~= nil then
				task.resume_request = request.resume_request
			end
		end
		table.insert(saved_tasks, task)

		::continue::
	end
	details.tasks = saved_tasks
end

--- @class SchedulerRequest
local scheduler_request = {}
scheduler_request.__index = scheduler_request

function scheduler_request:yield()
	coroutine.yield {
		resume_request = function()
			return true
		end,
	}
end

--- @param seconds number
function scheduler_request:sleep(seconds)
	assert(type(seconds) == "number" and seconds >= 0)

	local deadline = clock:now() + seconds
	coroutine.yield {
		resume_request = function()
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
		resume_request = function()
			local success = args.monitor()
			local timeout = clock:now() >= deadline
			return success or timeout
		end,
	}
	return clock:now() > deadline
end

return {
	--- @return Scheduler
	new = function()
		return setmetatable({
			details = {
				tasks = {},
			},
		}, scheduler)
	end,
	request = scheduler_request,
}
