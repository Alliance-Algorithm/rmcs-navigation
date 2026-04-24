local blackboard = require("blackboard").singleton()
local action = require("action")
local go_down_onestep = require("task.go-down-onestep")
local navigate_to_point = require("task.navigate-to-point")

local function is_same_point(a, b)
	return a.x == b.x and a.y == b.y
end

local function decide_escape_route()
	local rule = blackboard.rule
	local entry_point = rule.fluctuant_road_begin.ours
	local exit_point = rule.fluctuant_road_final.ours
	local resupply_point = rule.resupply_zone.ours

	local queue = blackboard.meta.navigate_point_queue
	if type(queue) ~= "table" then
		return "direct"
	end

	-- 从队尾开始回溯查询。
	for index = #queue, 1, -1 do
		local item = queue[index]
		if type(item) == "table" and type(item.x) == "number" and type(item.y) == "number" then
			-- 1. 先遇到补给点：直接回补给点
			if is_same_point(item, resupply_point) then
				return "direct"
			end

			-- 2. 先遇到起伏路段出口：先下台阶再回补给点
			if is_same_point(item, exit_point) then
				return "onestep"
			end

			-- 3. 先遇到起伏路段入口：先通过起伏路再回补给点
			if is_same_point(item, entry_point) then
				return "fluctuant_road"
			end
		end
	end

	-- 4. 全队列都没遇到起伏路段入口/出口或补给点：直接回补给点
	return "direct"
end

--- 回家补给：
--- - 先遇到补给点 resupply_zone：直接从当前位置回补给点。
--- - 先遇到起伏路段出口 fluctuant_road_final：先下一级台阶，再回补给点。
--- - 先遇到起伏路段入口 fluctuant_road_begin：原地开跟随到fluctuant_road_begin.ours回家。
--- - 全队列都没遇到起伏路段入口/出口或补给点：直接从当前位置回补给点。
--- @return boolean is_success
return function()
	local rule = blackboard.rule
	local entry_point = rule.fluctuant_road_begin.ours
	local resupply_zone = rule.resupply_zone.ours

	local route = decide_escape_route()
	local is_success = true
	if route == "onestep" then
		action:info("escape-to-home: 历史队列先命中起伏路段出口，下一级台阶回家")
		is_success = go_down_onestep(true)
		if not is_success then
			action:warn("escape-to-home: 下一级台阶失败")
			return false
		end
		
	elseif route == "fluctuant_road" then
		action:info("escape-to-home: 历史队列先命中起伏路段入口,原地开跟随到fluctuant_road_begin.ours回家")
		action:update_chassis_mode("LAUNCH_RAMP")
		local gimbal_yaw = math.pi
		action:info(string.format(
			"escape-fluctuant-road: LAUNCH_RAMP 云台朝向=%.3f rad",
			gimbal_yaw
		))
		action:update_gimbal_direction(gimbal_yaw)
		is_success = navigate_to_point(entry_point, {
			tolerance = 0.1,
			timeout = 10,
		})

		if not is_success then
			action:warn("escape-to-home: 通过起伏路失败")
			return false
		end
	else
		action:info("escape-to-home: 历史队列先命中补给点或未命中关键点，直接从当前位置回家")
	end

	action:update_chassis_mode("SPIN")
	is_success = navigate_to_point(resupply_zone, {
		tolerance = 0.15,
		timeout = 10,
	})
	if not is_success then
		action:warn("escape-to-home: 导航到补给点失败")
		return false
	end

	action:info("escape-to-home: 已抵达补给点")
	return true
end
