local action = require("action")
local bb = require("blackboard").singleton()

--- @class PatrolConfig
--- @field map Map
--- @field points MapPoint[]
--- @field dwell number 到点停留时长（秒）
--- @field timeout number 失败回退导航的超时（秒）
--- @field banner string 启动日志
--- @field resume? boolean true 时优先按 bb.context.current 续跑，否则取距机器人最近的点

--- @param config PatrolConfig
--- @return { loop: fun() }
return {
	new = function(config)
		local map = config.map
		local points = config.points
		local dwell = config.dwell
		local timeout = config.timeout

		local function start_index()
			if config.resume then
				local current = bb.context.current
				for index, point in ipairs(points) do
					if point == current then
						return index
					end
				end

				local nearest_index, nearest_distance = 1, math.huge
				for index, point in ipairs(points) do
					local dx = point.x - bb.user.x
					local dy = point.y - bb.user.y
					local distance = dx * dx + dy * dy
					if distance < nearest_distance then
						nearest_index, nearest_distance = index, distance
					end
				end
				return nearest_index
			end
			return 1
		end

		local intent = {}

		function intent:loop()
			action:info(config.banner)
			action:switch_motion_mode("attack")

			local count = #points
			local index = start_index()
			local begin = index

			while true do
				local target = points[index]
				action:cruise_slow_scan()
				while not action:follow_path(map, target) do
					action:cruise_slow_scan()
					action:navigate_until(bb.context.current, 0.5, timeout)
				end

				action:info("到达 " .. target.name .. "，停留 " .. dwell .. "s")
				action:cruise_fast_scan()
				action:dwell_scan(dwell)

				index = index % count + 1
				if index == begin then
					action:info("完成一圈巡逻")
				end
			end
		end

		return intent
	end,
}
