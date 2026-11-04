local action = require("action")
local bb = require("blackboard").singleton()

local MapRmul = require("map.rmul")
local Map, Points = MapRmul.map, MapRmul.points

-- 出击路线必须逐点执行：BFS 按跳数取最短，直接 search 到巡逻区会绕 kEscape 而不钻洞
local kRoute = {
	Points.kHoleBegin,
	Points.kHoleEnd,
	Points.kPatrol1,
}

-- 中央控制区四角，顺时针 1 -> 2 -> 3 -> 4
local kPatrol = {
	Points.kPatrol1,
	Points.kPatrol2,
	Points.kPatrol3,
	Points.kPatrol4,
}

local kPatrolDwell = 3 -- 秒，到角点后短暂停车扫描

local intent = {}

local function patrol_index_for(point)
	for index, candidate in ipairs(kPatrol) do
		if candidate == point then
			return index
		end
	end
	return nil
end

function intent:loop()
	action:info("出击：钻狗洞，前往中央控制区 1 号点")
	action:switch_motion_mode("normal")

	-- 如果从巡逻点重启，直接接着巡逻；否则从当前节点续跑出击路线
	local patrol_start = patrol_index_for(bb.context.current)
	if patrol_start == nil then
		-- 从当前所在节点之后续跑，补给回来或意图重启都从头钻洞
		local index = 1
		for i, point in ipairs(kRoute) do
			if point == bb.context.current then
				index = i + 1
			end
		end

		while index <= #kRoute do
			local target = kRoute[index]

			-- 钻洞前不迎战，尽快拿到 1.5 倍热量增益；出洞后途中识别到敌方即停车迎战
			bb.context.engage_on_route = target == Points.kPatrol1

			action:cruise_slow_scan()
			if action:follow_path(Map, target, { merge = false }) then
				index = index + 1
			elseif target ~= Points.kHoleEnd then
				-- 兜底：返回上一个确认点重试；狗洞边失败时由 cross_hole 自行重新对准洞口
				action:navigate_until(bb.context.current, 0.5, 10)
			end
		end
		patrol_start = 1 -- 刚到达 1 号点
	end

	-- 巡逻模式：1 -> 2 -> 3 -> 4 -> 1，顺时针；到点短暂停车扫描
	bb.context.engage_on_route = true
	action:switch_motion_mode("attack")
	action:info("开始顺时针巡逻：1 -> 2 -> 3 -> 4")

	local index = patrol_start % #kPatrol + 1
	while true do
		local target = kPatrol[index]
		action:cruise_slow_scan()
		while not action:follow_path(Map, target, { merge = false }) do
			action:cruise_slow_scan()
			action:navigate_until(bb.context.current, 0.5, 10)
		end

		action:cruise_fast_scan()
		action:dwell_scan(kPatrolDwell)
		index = index % #kPatrol + 1
	end
end

return intent
