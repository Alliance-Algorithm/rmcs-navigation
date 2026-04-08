---@class Edge
---@field private _signal any
---@field private _last any|nil
local Edge = {}
Edge.__index = Edge

local function new_edge(signal)
	return setmetatable({ _signal = signal, _last = nil }, Edge)
end

function Edge:spin(value)
	local triggered = (value == self._signal) and (value ~= self._last)
	self._last = value
	return triggered
end

function Edge:reset()
	self._last = nil
end

---@class EdgeEntry
---@field getter fun(): any
---@field edge Edge
---@field callback fun()

---@class Edges
---@field private _entries EdgeEntry[]
local Edges = {}
Edges.__index = Edges

---@param getter fun(): any
---@param signal any
---@param callback fun()
---@return Edges
function Edges:on(getter, signal, callback)
	self._entries[#self._entries + 1] = {
		getter = getter,
		edge = new_edge(signal),
		callback = callback,
	}
	return self
end

function Edges:spin()
	for _, entry in ipairs(self._entries) do
		if entry.edge:spin(entry.getter()) then
			entry.callback()
		end
	end
end

function Edges:reset()
	for _, entry in ipairs(self._entries) do
		entry.edge:reset()
	end
end

return {
	--- @return Edges
	new = function()
		return setmetatable({ _entries = {} }, Edges)
	end,
}
