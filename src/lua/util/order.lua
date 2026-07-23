local clock = require("util.clock")

---@class OrderEntry
---@field sequence any[]
---@field callback fun()
---@field step integer
---@field last_time number|nil
---@field pending_at number|nil

---@class Order
---@field private _getter fun(): any
---@field private _interval number
---@field private _entries OrderEntry[]
---@field private _last_value any
local Order = {}
Order.__index = Order

local function reset_entry(entry)
	entry.step = 0
	entry.last_time = nil
end

local function prime_entry(entry, value)
	reset_entry(entry)
	if value == entry.sequence[1] then
		entry.step = 1
	end
end

--- short 是否为 long 的严格前缀
local function is_strict_prefix(short_seq, long_seq)
	if #short_seq >= #long_seq then
		return false
	end
	for i = 1, #short_seq do
		if short_seq[i] ~= long_seq[i] then
			return false
		end
	end
	return true
end

--- 是否存在以 entry 为严格前缀、且仍在匹配/已完成的更长序列
local function find_longer_contender(entry, entries)
	local best = nil
	for _, other in ipairs(entries) do
		if other ~= entry and is_strict_prefix(entry.sequence, other.sequence) then
			local active = other.pending_at ~= nil or other.step >= #entry.sequence
			if active and (best == nil or #other.sequence > #best.sequence) then
				best = other
			end
		end
	end
	return best
end

--- 推进 entry；返回是否在本步完成整段序列（不触发 callback）
local function advance_entry(entry, value, now)
	local first = entry.sequence[1]
	if entry.step == 0 then
		prime_entry(entry, value)
		if #entry.sequence == 1 and value == first then
			return true
		end
		return false
	end

	local expected = entry.sequence[entry.step + 1]
	if value == expected then
		entry.step = entry.step + 1
		if entry.step == #entry.sequence then
			return true
		end
		if entry.step >= 2 then
			entry.last_time = now
		end
		return false
	end

	prime_entry(entry, value)
	if #entry.sequence == 1 and value == first then
		return true
	end
	return false
end

---@param sequence any[]
---@param callback fun()
---@return Order
function Order:on(sequence, callback)
	local entry = {
		sequence = sequence,
		callback = callback,
		step = 0,
		last_time = nil,
		pending_at = nil,
	}
	prime_entry(entry, self._last_value)
	self._entries[#self._entries + 1] = entry
	return self
end

function Order:spin()
	local value = self._getter()
	local now = clock:now()
	local value_changed = value ~= self._last_value
	if value_changed then
		self._last_value = value
	end

	for _, entry in ipairs(self._entries) do
		if entry.step > 1 and entry.last_time ~= nil and now - entry.last_time > self._interval then
			reset_entry(entry)
		end
	end

	if value_changed then
		for _, entry in ipairs(self._entries) do
			if advance_entry(entry, value, now) then
				entry.pending_at = now
				prime_entry(entry, value)
			end
		end
	end

	local to_fire = {}
	for _, entry in ipairs(self._entries) do
		if entry.pending_at ~= nil then
			local contender = find_longer_contender(entry, self._entries)
			if contender == nil then
				to_fire[#to_fire + 1] = entry
			elseif contender.pending_at ~= nil then
				entry.pending_at = nil
			end
		end
	end

	table.sort(to_fire, function(a, b)
		return #a.sequence > #b.sequence
	end)

	local fired = {}
	for _, entry in ipairs(to_fire) do
		local suppressed = false
		for _, done in ipairs(fired) do
			if is_strict_prefix(entry.sequence, done.sequence) then
				suppressed = true
				break
			end
		end
		entry.pending_at = nil
		if not suppressed then
			entry.callback()
			fired[#fired + 1] = entry
		end
	end
end

function Order:reset()
	self._last_value = self._getter()
	for _, entry in ipairs(self._entries) do
		entry.pending_at = nil
		prime_entry(entry, self._last_value)
	end
end

return {
	---@param getter fun(): any
	---@param interval number
	---@return Order
	new = function(getter, interval)
		return setmetatable({
			_getter = getter,
			_interval = interval,
			_entries = {},
			_last_value = getter(),
		}, Order)
	end,
}
