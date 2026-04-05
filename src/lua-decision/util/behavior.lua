local SUCCESS = "success"
local FAILURE = "failure"
local RUNNING = "running"

local M = {}
M.SUCCESS = SUCCESS
M.FAILURE = FAILURE
M.RUNNING = RUNNING

function M.condition(fn)
	return {
		label = "condition",
		exec = function(_)
			return fn() and SUCCESS or FAILURE
		end,
	}
end

--- Leaf: user logic, may call ctx:sleep() / ctx:wait_until().
--- Must return SUCCESS or FAILURE.
---@param fn fun(ctx: Task): string
function M.action(fn)
	return {
		label = "action",
		exec = function(ctx)
			return fn(ctx)
		end,
	}
end

function M.sequence(childrens)
	return {
		label = "sequence",
		exec = function(ctx)
			for _, child in ipairs(childrens) do
				local status = child.exec(ctx)
				if status ~= SUCCESS then
					return status
				end
			end
			return SUCCESS
		end,
	}
end

function M.selector(childrens)
	return {
		label = "selector",
		exec = function(ctx)
			for _, child in ipairs(childrens) do
				local status = child.exec(ctx)
				if status ~= FAILURE then
					return status
				end
			end
			return FAILURE
		end,
	}
end

---@class Behavior
---@field private _root table
---@field private _io_context IoContext|nil
---@field private _handle Handle|nil
local Behavior = {}
Behavior.__index = Behavior

--- Create a behavior tree from a root node.
---@param root table
---@return Behavior
function M.new(root)
	return setmetatable({
		_root = root,
		_io_context = nil,
		_handle = nil,
	}, Behavior)
end

--- Bind to an IoContext and start running.
---@param io_context IoContext
function Behavior:bind(io_context)
	self._io_context = io_context
	self:reset()
end

--- Kill current coroutine and restart the tree from scratch.
function Behavior:reset()
	if self._handle then
		self._handle:cancel()
		self._handle = nil
	end
	if self._io_context then
		local root = self._root
		self._handle = self._io_context:spawn(function(ctx)
			while true do
				root.exec(ctx)
				ctx:yield()
			end
		end)
	end
end

return M
