local blackboard = require("blackboard").singleton()
local action = require("action")
local go_down_onestep = require("task.go-down-onestep")
local navigate_to_point = require("task.navigate-to-point")

local function is_same_point(a, b)
	return a.x == b.x and a.y == b.y
end

local function decide_escape_route(ours_zone)
	local rule = blackboard.rule
	local exit_point
	local resupply_point
	if ours_zone then
		exit_point = rule.fluctuant_road_final.ours
		resupply_point = rule.resupply_zone.ours
	else
		exit_point = rule.fluctuant_road_final.them
		resupply_point = rule.resupply_zone.them
	end

	local queue = blackboard.meta.navigate_point_queue
	if type(queue) ~= "table" then
		return "direct"
	end

	-- 从队尾（最近一次）开始回溯查询。
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
		end
	end

	-- 3. 全队列都没遇到起伏路段出口或补给点：直接回补给点
	return "direct"
end

--- 回家补给：
--- - 先遇到补给点 resupply_zone：直接从当前位置回补给点。
--- - 先遇到起伏路段出口 fluctuant_road_final：先下一级台阶，再回补给点。
--- - 全队列都没遇到起伏路段出口或补给点：直接从当前位置回补给点。
--- @param ours_zone boolean
--- @return boolean is_success
return function(ours_zone)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")

	local rule = blackboard.rule
	local resupply_zone
	if ours_zone then
		resupply_zone = rule.resupply_zone.ours
	else
		resupply_zone = rule.resupply_zone.them
	end

	local route = decide_escape_route(ours_zone)
	local is_success = true
	if route == "onestep" then
		action:info("escape-to-home: 历史队列先命中起伏路段出口，按原有逻辑回家")
		is_success = go_down_onestep(ours_zone)
		if not is_success then
			action:warn("escape-to-home: 下一级台阶失败")
			return false
		end
	else
		action:info("escape-to-home: 历史队列先命中补给点或未命中关键点，直接从当前位置回家")
	end

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
