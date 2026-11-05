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

--- 沿地图逐条边前往 target，失败则返回上一个确认点重试。
--- 返回 false 表示途中已经补满，调用方应结束补给。
--- 不合并普通边：狗洞在全局地图上是空地，合并后直接导航到远处目标，全局规划可能穿洞
local function reach(target)
	while true do
		local ok, interrupted = action:follow_path(Map, target, { merge = false, check = is_supplied })
		if interrupted then
			return false
		end
		if ok then
			return true
		end
		action:navigate_until(bb.context.current, 0.25, 10)
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
	if bb.condition.near(Points.kBorn, 0.5) then
		bb.context.current = Points.kBorn
	end

	-- kHoleBegin、kHoleEnd、kPatrol1 被击杀：沿狗洞折返。其余巡逻点绕 kEscape
	local current = bb.context.current

	local supplied_on_route = false
	if current == Points.kPatrol1 then
		supplied_on_route = not reach(Points.kHoleEnd)
		current = bb.context.current
	end
	if not supplied_on_route and current == Points.kHoleEnd then
		supplied_on_route = not reach(Points.kHoleBegin)
		current = bb.context.current
	end

	if
		not supplied_on_route
		and current ~= Points.kBegin
		and current ~= Points.kHoleBegin
		and current ~= Points.kBorn
	then
		supplied_on_route = not reach(Points.kEscape)
	end
	if not supplied_on_route then
		supplied_on_route = not reach(Points.kBorn)
	end

	if supplied_on_route then
		action:info("途中补给完成，直接返回")
	else
		-- current 已是 kBorn 时 follow_path 不下发目标，导航可能仍指向上一个意图的点
		action:navigate(Points.kBorn)
		-- 到达补给区：等待补给完成或到达时限
		local timeout = request:wait_until {
			monitor = is_supplied,
			timeout = kSupplyInterval,
		}
		action:info(timeout and "补给时限已到，返回战场" or "补给完成，返回战场")
	end
	bb.context.unhealth = false
	action:update_supercap_boost(false)
end

return intent
