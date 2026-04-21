local action = require("action")
local crossing_road_zone_train = require("task.crossing-road-zone-train")

--- 训练专用：
--- 直接调用 crossing-road-zone-train 通过公路区
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	action:info("开始start-cruise-train")

	local ok = crossing_road_zone_train(ours_zone)
	if not ok then
		action:warn("start-cruise-train: 通过公路区训练路线失败")
		return false
	end

	action:info("start-cruise-train: 已完成公路区训练路线")
	return true
end
