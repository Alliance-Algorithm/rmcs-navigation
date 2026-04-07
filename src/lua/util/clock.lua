--- @class Clock
--- @field private _timestamp number
--- @field private _ready boolean
local clock = {
	_timestamp = 0,
	_ready = false,
}

--- @param timestamp number
--- @return number
function clock:update(timestamp)
	assert(type(timestamp) == "number", "clock timestamp must be a number")

	self._timestamp = timestamp
	self._ready = true
	return self._timestamp
end

--- @return number
function clock:now()
	return self._timestamp
end

--- @return boolean
function clock:is_ready()
	return self._ready
end

--- @param timestamp? number
function clock:reset(timestamp)
	if timestamp == nil then
		self._timestamp = 0
		self._ready = false
		return
	end

	assert(type(timestamp) == "number", "clock reset timestamp must be a number")

	self._timestamp = timestamp
	self._ready = true
end

return clock
