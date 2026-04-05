---@class Interrupt
---@field private _behavior Behavior
---@field private _entries InterruptEntry[]
local Interrupt = {}
Interrupt.__index = Interrupt

---@class InterruptEntry
---@field getter fun(): boolean
---@field prev boolean|nil

--- Create an interrupt monitor bound to a behavior tree.
---@param behavior Behavior
---@return Interrupt
local function new_interrupt(behavior)
	return setmetatable({
		_behavior = behavior,
		_entries = {},
	}, Interrupt)
end

--- Register an interrupt condition.
--- When getter() transitions from false to true, behavior:reset() is called.
---@param getter fun(): boolean
---@return Interrupt
function Interrupt:on(getter)
	self._entries[#self._entries + 1] = {
		getter = getter,
		prev = nil,
	}
	return self
end

--- Check all interrupt conditions. Call this every tick.
function Interrupt:spin()
	for _, entry in ipairs(self._entries) do
		local current = entry.getter()
		local prev = entry.prev
		entry.prev = current

		-- Rising edge: was not true, now is true
		if current and not prev then
			self._behavior:reset()
			return -- one reset per tick is enough
		end
	end
end

return {
	new = new_interrupt,
}
