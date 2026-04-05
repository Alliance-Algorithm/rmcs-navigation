---
--- FsmHandle
---

--- @class FsmHandle
--- @field private _fsm Fsm
local Handle = {}
Handle.__index = Handle

--- @param fsm Fsm
--- @return FsmHandle
function Handle:new(fsm)
	return setmetatable({
		_fsm = fsm,
	}, self)
end

--- @param status string
function Handle:set_next(status)
	self._fsm._pending_next = status
end

--- @return string|nil
function Handle:last_state()
	return self._fsm._last_state
end

---
--- Fsm
---

--- @class FsmStateEntry
--- @field enter fun()
--- @field event fun(handle: FsmHandle)

--- @class FsmStateConfig
--- @field state string
--- @field enter? fun()
--- @field event fun(handle: FsmHandle)

--- @class Fsm
--- @field _state_mapping table<string, FsmStateEntry>
--- @field _current_event FsmStateEntry|nil
--- @field _current_state string
--- @field _last_state string|nil
--- @field _pending_next string|nil
--- @field _handle FsmHandle
local Fsm = {}
Fsm.__index = Fsm

--- @param start_state string
--- @return Fsm
function Fsm:new(start_state)
	local fsm = setmetatable({
		_state_mapping = {},
		_current_event = nil,
		_current_state = start_state,
		_last_state = nil,
		_pending_next = nil,
		_handle = nil,
	}, self)

	fsm._handle = Handle:new(fsm)

	return fsm
end

function Fsm:spin_once()
	if self._current_event == nil then
		self._current_event = self._state_mapping[self._current_state]
		assert(self._current_event ~= nil, "state is not registered")
		self._current_event.enter()
	end

	self._pending_next = nil

	self._current_event.event(self._handle)

	if self._pending_next ~= nil and self._pending_next ~= self._current_state then
		self._last_state = self._current_state
		self._current_state = self._pending_next
		self._current_event = nil
	end
end

--- @param state string
function Fsm:start_on(state)
	assert(state ~= nil, "state is required")
	assert(self._state_mapping[state] ~= nil, "state is not registered")

	self._current_state = state
	self._current_event = nil
	self._last_state = nil
	self._pending_next = nil
end

--- @param config FsmStateConfig
function Fsm:use(config)
	local state = config.state
	assert(state ~= nil, "state is required")
	assert(config.event ~= nil, "event is required")

	self._state_mapping[state] = {
		enter = config.enter or function() end,
		event = config.event,
	}
end

--- @param states table<string, string>
--- @return boolean
function Fsm:init_ready(states)
	if states == nil or self._current_state == nil then
		return false
	end

	local has_current = false

	for _, state in pairs(states) do
		if state == self._current_state then
			has_current = true
		end

		local config = self._state_mapping[state]
		if config == nil or config.event == nil then
			return false
		end
	end

	return has_current
end

return Fsm
