--- @class IoHandle
--- @field private _context IoContext
local IoHandle = {}
IoHandle.__index = IoHandle

function IoHandle:sleep(seconds)
	coroutine.yield("sleep", seconds)
end

function IoHandle:yield()
	coroutine.yield("yield")
end

--- @param opts fun(): boolean | { pred: fun(): boolean, timeout: number|nil, interval: number|nil }
function IoHandle:wait_until(opts)
	if type(opts) == "function" then
		return coroutine.yield("wait_until", {
			pred = opts,
			timeout_at = nil,
			interval = nil,
			next_check_at = nil,
		})
	end

	assert(type(opts) == "table", "wait_until expects a predicate or options table")
	assert(type(opts.pred) == "function", "wait_until expects opts.pred to be a function")

	local timeout = opts.timeout
	local interval = opts.interval

	if timeout ~= nil then
		assert(type(timeout) == "number" and timeout > 0, "wait_until timeout must be a positive number")
	end
	if interval ~= nil then
		assert(type(interval) == "number" and interval > 0, "wait_until interval must be a positive number")
	end

	return coroutine.yield("wait_until", {
		pred = opts.pred,
		timeout_at = timeout and (self._context._now + timeout) or nil,
		interval = interval,
		next_check_at = nil,
	})
end

function IoHandle:now()
	return self._context._now
end

--- @class TaskEntry
--- @field co thread
--- @field kind string
--- @field value any
--- @field elapsed number
--- @field cancelled boolean
--- @field done boolean

--- @class SpawnHandle
--- @field private _entry TaskEntry
local SpawnHandle = {}
SpawnHandle.__index = SpawnHandle

function SpawnHandle:cancel()
	self._entry.cancelled = true
end

--- @return boolean
function SpawnHandle:is_alive()
	local e = self._entry
	return not e.cancelled and not e.done
end

--- @class IoContext
--- @field private _tasks TaskEntry[]
--- @field private _last_timestamp number|nil
--- @field _now number
local IoContext = {}
IoContext.__index = IoContext

--- @return IoContext
local function new_io_context()
	return setmetatable({
		_tasks = {},
		_last_timestamp = nil,
		_now = 0,
	}, IoContext)
end

--- @param fn fun(handle: IoHandle)
--- @return SpawnHandle
function IoContext:spawn(fn)
	local handle = setmetatable({ _context = self }, IoHandle)
	local co = coroutine.create(function()
		fn(handle)
	end)

	local ok, kind, value = coroutine.resume(co)
	if not ok then
		print("[io_context] task error: " .. tostring(kind))
		return setmetatable({ _entry = { cancelled = true, done = true } }, SpawnHandle)
	end

	--- @type TaskEntry
	local entry = {
		co = co,
		kind = kind or "",
		value = value,
		elapsed = 0,
		cancelled = false,
		done = false,
	}

	if coroutine.status(co) ~= "dead" then
		self._tasks[#self._tasks + 1] = entry
	else
		entry.done = true
	end

	return setmetatable({ _entry = entry }, SpawnHandle)
end

--- @param timestamp number
function IoContext:spin(timestamp)
	local dt = 0
	if self._last_timestamp then
		dt = timestamp - self._last_timestamp
	end
	self._last_timestamp = timestamp
	self._now = timestamp

	local alive = {}
	for _, entry in ipairs(self._tasks) do
		if entry.cancelled then
			-- skip
		else
			local ready = false

			if entry.kind == "yield" then
				ready = true
			elseif entry.kind == "sleep" then
				entry.elapsed = entry.elapsed + dt
				ready = entry.elapsed >= entry.value
			elseif entry.kind == "wait_until" then
				local wait = entry.value
				if wait.timeout_at ~= nil and timestamp >= wait.timeout_at then
					local ok, kind, value = coroutine.resume(entry.co, false, "timeout")
					if not ok then
						print("[io_context] task error: " .. tostring(kind))
						entry.done = true
					elseif coroutine.status(entry.co) ~= "dead" then
						entry.kind = kind
						entry.value = value
						entry.elapsed = 0
						alive[#alive + 1] = entry
					else
						entry.done = true
					end
					goto continue
				end

				if wait.next_check_at ~= nil and timestamp < wait.next_check_at then
					alive[#alive + 1] = entry
					goto continue
				end

				ready = wait.pred()
				if not ready and wait.interval ~= nil then
					wait.next_check_at = timestamp + wait.interval
				end
			end

			if ready then
				local ok, kind, value
				if entry.kind == "wait_until" then
					ok, kind, value = coroutine.resume(entry.co, true, nil)
				else
					ok, kind, value = coroutine.resume(entry.co)
				end
				if not ok then
					print("[io_context] task error: " .. tostring(kind))
					entry.done = true
				elseif coroutine.status(entry.co) ~= "dead" then
					entry.kind = kind
					entry.value = value
					entry.elapsed = 0
					alive[#alive + 1] = entry
				else
					entry.done = true
				end
			else
				alive[#alive + 1] = entry
			end

			::continue::
		end
	end
	self._tasks = alive
end

function IoContext:clear()
	for _, entry in ipairs(self._tasks) do
		entry.cancelled = true
	end
	self._tasks = {}
end

return {
	new = new_io_context,
}
