local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kCruiseInterval = 3 -- 秒，每个节点停留时长

local full_map_points = {
	Points.kOrigin, -- kBegin
	Points.kSelfHighlandBegin,
	Points.kAttackRune,
	Points.kAttackOutpost,
	Points.kSelfDoubleStepsBegin,
	Points.kSelfStepBegin,
	Points.kSelfStepFinal,
	Points.kSelfSlopeBegin,
	Points.kNearSelfOutpost,
	Points.kSelfDoubleStepsFinal,
	Points.kThemDoubleStepsFinal,
	Points.kNearThemOutpost,
	Points.kThemStepFinal,
	Points.kThemStepBegin,
	Points.kThemDoubleStepsBegin,
	Points.kThemHighlandBegin,
	Points.kThemHighlandFinal,
}

local intent = {}

local function cruise_leg(from, to)
	for _, step in ipairs(Map:search(from, to)) do
		if not step.run() then
			action:warn("巡游段失败: " .. step.begin_name .. " -> " .. step.final_name)
			return false
		end
		bb.context.current = step.final_point
	end
	return true
end

function intent:loop()
	action:info("全图游走")
	action:switch_motion_mode("attack")

	local count = #full_map_points
	local index = 1

	while true do
		local from = full_map_points[index]
		local to = full_map_points[index % count + 1]

		action:gimbal_toward(0, 0)
		if cruise_leg(from, to) then
			action:gimbal_scan(0, 0)
			request:sleep(kCruiseInterval)
			index = index % count + 1
		else
			action:warn("巡游段重试: " .. from.name .. " -> " .. to.name)
			action:gimbal_toward(0, 0)
			action:navigate(from)
			request:wait_until {
				monitor = function()
					return bb.condition.near(from, 0.5)
				end,
				timeout = 10,
			}
		end
	end
end

return intent
