local action = require("action")
local cruise_in_central_highlands = require("task.cruise-in-central-highland.cruise-in-central-highlands")

--- 持续巡航：在中央高地两点间循环巡航（通常为长期运行）。
--- @param ours_zone boolean
--- @param switch_interval number 中央高地巡航切换周期（秒）
--- @return boolean is_success
return function(ours_zone, switch_interval)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(switch_interval) == "number", "switch_interval should be a number")
	assert(switch_interval > 0, "switch_interval should be positive")

	action:info("keep-cruise: 进入中央高地持续巡航")
	local ok = cruise_in_central_highlands(ours_zone, switch_interval)
	if not ok then
		action:warn("keep-cruise: 中央高地巡航导航失败")
		return false
	end

	return true
end
