---
--- FsmHandle
---

--- @class FsmHandleDetails
--- @field fsm Fsm

--- @class FsmHandle
--- @field details FsmHandleDetails
local handle = {}
handle.__index = handle

--- @param fsm Fsm
--- @return FsmHandle
function handle:new(fsm)
	return setmetatable({
		details = {
			fsm = fsm,
		},
	}, self)
end

--- @param status string
function handle:set_next(status)
	local fsm = self.details.fsm
	fsm.details.pending_next = status
end

--- @return string|nil
function handle:last_state()
	local fsm = self.details.fsm
	return fsm.details.last_state
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

--- @class FsmDetails
--- @field state_mapping table<string, FsmStateEntry>
--- @field current_event FsmStateEntry|nil
--- @field current_state string
--- @field last_state string|nil
--- @field pending_next string|nil
--- @field handle FsmHandle

--- @class Fsm
--- @field details FsmDetails
local Fsm = {}
Fsm.__index = Fsm

--- @param start_state string
--- @return Fsm
function Fsm:new(start_state)
	local fsm = setmetatable({}, self)

	fsm.details = {
		state_mapping = {},
		current_event = nil,
		current_state = start_state,
		last_state = nil,
		pending_next = nil,
		handle = handle:new(fsm),
	}
	return fsm
end

function Fsm:spin_once()
	local details = self.details

	if details.current_event == nil then
		details.current_event = details.state_mapping[details.current_state]
		assert(details.current_event ~= nil, "state is not registered")
		details.current_event.enter()
	end

	details.pending_next = nil

	details.current_event.event(details.handle)

	if details.pending_next ~= nil and details.pending_next ~= details.current_state then
		details.last_state = details.current_state
		details.current_state = details.pending_next
		details.current_event = nil
	end
end

--- @param state string
function Fsm:start_on(state)
	local details = self.details

	assert(state ~= nil, "state is required")
	assert(details.state_mapping[state] ~= nil, "state is not registered")

	details.current_state = state
	details.current_event = nil
	details.last_state = nil
	details.pending_next = nil
end

--- @param config FsmStateConfig
function Fsm:use(config)
	local details = self.details

	local state = config.state
	assert(state ~= nil, "state is required")
	assert(config.event ~= nil, "event is required")

	details.state_mapping[state] = {
		enter = config.enter or function() end,
		event = config.event,
	}
end

--- @param states table<string, string>
--- @return boolean
function Fsm:init_ready(states)
	local details = self.details

	if states == nil or details.current_state == nil then
		return false
	end

	local has_current = false

	for _, state in pairs(states) do
		if state == details.current_state then
			has_current = true
		end

		local config = details.state_mapping[state]
		if config == nil or config.event == nil then
			return false
		end
	end

	return has_current
end

return Fsm
