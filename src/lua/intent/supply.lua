local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()

local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kHealthReady = 350
local kBulletReady = 95
local kSupplyInterval = 6 -- 秒，到达补给区后最多停留时长

local intent = {}

local function is_supplied()
	return bb.user.health >= kHealthReady and bb.user.bullet >= kBulletReady
end

function intent:loop()
	action:info("需要补给，回家")
	action:info("health: " .. bb.user.health)
	action:info("bullet: " .. bb.user.bullet)

	-- 兜底任务：永不放弃。失败则返回上一个确认点，重新搜索重试
	while true do
		local failed = false

		for index, path in ipairs(Map:search(bb.context.current, Points.kHome)) do
			action:info("Execute path task: " .. index .. " " .. path.begin_name .. " -> " .. path.final_name)
			if not path.run() then
				action:warn("路径任务失败，返回 " .. bb.context.current.name .. " 重试")
				failed = true
				break
			end
			bb.context.current = path.final_point -- 每条边成功后推进

			if is_supplied() then
				action:info("途中补给完成，直接返回")
				bb.context.hint_intent = bb.context.intent_to_return
				return
			end
		end

		if not failed then
			break
		end

		-- 返回上一个确认点（容差/时限与 rough_navigate 一致，局部内联）
		action:navigate(bb.context.current)
		request:wait_until {
			monitor = function()
				return bb.condition.near(bb.context.current, 0.5)
			end,
			timeout = 10,
		}
	end

	-- 到达补给区：等待补给完成或到达时限
	local timeout = request:wait_until {
		monitor = is_supplied,
		timeout = kSupplyInterval,
	}
	action:info(timeout and "补给时限已到，返回战场" or "补给完成，返回战场")

	bb.context.hint_intent = bb.context.intent_to_return
end

return intent
