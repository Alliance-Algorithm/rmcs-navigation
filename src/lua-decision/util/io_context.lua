---@class Task
---@field private _co thread
local Task = {}
Task.__index = Task

function Task:sleep(seconds)
	coroutine.yield("sleep", seconds)
end

function Task:yield()
	coroutine.yield("yield")
end

---@param predicate fun(): boolean
function Task:wait_until(predicate)
	coroutine.yield("wait_until", predicate)
end

---@class TaskEntry
---@field co thread
---@field kind string
---@field value any
---@field elapsed number
---@field cancelled boolean
---@field done boolean

---@class Handle
---@field private _entry TaskEntry
local Handle = {}
Handle.__index = Handle

function Handle:cancel()
	self._entry.cancelled = true
end

---@return boolean
function Handle:is_alive()
	local e = self._entry
	return not e.cancelled and not e.done
end

---@class IoContext
---@field private _tasks TaskEntry[]
---@field private _last_timestamp number|nil
local IoContext = {}
IoContext.__index = IoContext

---@return IoContext
local function new_io_context()
	return setmetatable({
		_tasks = {},
		_last_timestamp = nil,
	}, IoContext)
end

---@param fn fun(task: Task)
---@return Handle
function IoContext:spawn(fn)
	local task = setmetatable({}, Task)
	local co = coroutine.create(function()
		fn(task)
	end)

	local ok, kind, value = coroutine.resume(co)
	if not ok then
		print("[io_context] task error: " .. tostring(kind))
		return setmetatable({ _entry = { cancelled = true, done = true } }, Handle)
	end

	---@type TaskEntry
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

	return setmetatable({ _entry = entry }, Handle)
end

---@param timestamp number
function IoContext:spin(timestamp)
	local dt = 0
	if self._last_timestamp then
		dt = timestamp - self._last_timestamp
	end
	self._last_timestamp = timestamp

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
				ready = entry.value()
			end

			if ready then
				local ok, kind, value = coroutine.resume(entry.co)
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
