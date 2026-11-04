local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local MapRmul = require("map.rmul")
local Map, Points = MapRmul.map, MapRmul.points

local kHealthReady = 400
local kBulletReady = 80
local kSupplyInterval = 60 -- 秒，到达补给区后最多停留时长

local intent = {}

local function is_supplied()
	return bb.user.health >= kHealthReady and bb.user.bullet >= kBulletReady
end

--- 沿地图逐条边前往 target，失败则返回上一个确认点重试
--- 不合并普通边：狗洞在全局地图上是空地，合并后直接导航到远处目标，全局规划可能穿洞
local function reach(target)
	while not action:follow_path(Map, target, { merge = false }) do
		action:navigate_until(bb.context.current, 0.5, 10)
	end
end

function intent:loop()
	action:info("需要补给，回家")
	action:info("health: " .. bb.user.health)
	action:info("bullet: " .. bb.user.bullet)

	bb.context.engage_on_route = false
	action:switch_motion_mode("attack")
	action:cruise_slow_scan()

	-- 阵亡复活后已在补给区，但 current 仍停在阵亡前的节点
	if bb.condition.near(Points.kBorn, 1.0) then
		bb.context.current = Points.kBorn
	end

	-- 返回不走狗洞：尚未钻洞时直接回家，否则强制经过 kEscape
	local current = bb.context.current

	-- 刚出洞就要补给时，先回 1 号点；否则图搜索的最短路可能回头穿狗洞
	if current == Points.kHoleEnd then
		reach(Points.kPatrol1)
		current = bb.context.current
	end

	if current ~= Points.kBegin and current ~= Points.kHoleBegin and current ~= Points.kBorn then
		reach(Points.kEscape)
	end
	reach(Points.kBorn)

	-- 到达补给区：等待补给完成或到达时限
	local timeout = request:wait_until {
		monitor = is_supplied,
		timeout = kSupplyInterval,
	}
	bb.context.unhealth = false
	action:info(timeout and "补给时限已到，返回战场" or "补给完成，返回战场")
end

return intent
