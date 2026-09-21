local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()

local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kHealthReady = 350
local kBulletReady = 80
local kSupplyInterval = 75 -- 秒，到达补给区后最多停留时长
local kNavScanSpeed = 4.5  -- 秒/弧度，导航中云台慢扫（约 20s 一圈）

local intent = {}

local function is_supplied()
	return bb.user.health >= kHealthReady and bb.user.bullet >= kBulletReady
end

function intent:loop()
	action:info("需要补给，回家")
	action:info("health: " .. bb.user.health)
	action:info("bullet: " .. bb.user.bullet)

	-- supply 开始时关闭符文追踪
	action:update_track_rune(false)

	-- 兜底任务：永不放弃。失败则返回上一个确认点，重新搜索重试
	while true do
		action:set_gimbal_yt(kNavScanSpeed)
		action:gimbal_scan(0, 0)

		local ok, interrupted = action:follow_path(Map, Points.kHome, is_supplied)
		if interrupted then
			action:info("途中补给完成，直接返回")
			blackboard.context.unhealth = false
			-- supply 结束前关闭超级电容
			action:update_supercap_boost(false)
			return
		end
		if ok then
			break
		end

		-- 返回上一个确认点（容差/时限与 rough_navigate 一致，局部内联）
		action:set_gimbal_yt(kNavScanSpeed)
		action:gimbal_scan(0, 0)
		action:navigate_until(bb.context.current, 0.5, 10)
	end

	-- 到达补给区：等待补给完成或到达时限
	local timeout = request:wait_until {
		monitor = is_supplied,
		timeout = kSupplyInterval,
	}
	blackboard.context.unhealth = false
	action:info(timeout and "补给时限已到，返回战场" or "补给完成，返回战场")
	-- supply 结束前关闭超级电容
	action:update_supercap_boost(false)
end

return intent
