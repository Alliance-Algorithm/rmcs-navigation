local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local intent = {}

function intent:loop()
	action:info("前往敌方基地")
	action:switch_motion_mode("attack")
	action:cruise_slow_scan()

	local target = Points.kAttackBase
	while true do
		local failed = false
		for i, path in ipairs(Map:search(bb.context.current, target)) do
			action:info("Execute path task: " .. i .. " " .. path.begin_name .. " -> " .. path.final_name)
			if not path.run() then
				action:warn("路径任务失败，返回 " .. bb.context.current.name .. " 重试")
				failed = true
				break
			end
			bb.context.current = path.final_point
		end
		if not failed then
			break
		end

		action:navigate(bb.context.current)
		request:wait_until {
			monitor = function()
				return bb.condition.near(bb.context.current, 0.5)
			end,
			timeout = 10,
		}
	end
	action:info("已到达 " .. target.name)

	action:switch_motion_mode("attack")
	action:cruise_fast_scan()
	request:sleep(300) -- 基本等于永久停留
end

return intent
