local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kStayDuration = 10 -- 秒，到达哨站后停留时长

local intent = {}

function intent:loop()
	action:info("前往敌方前哨站")
	action:switch_motion_mode("normal")
	action:gimbal_toward(0, 0.3)

	blackboard.context.powered_move = true

	local target = Points.kAttackOutpost
	while not action:follow_path(Map, target) do
		action:navigate_until(bb.context.current, 0.5, 10)
	end
	action:info("已到达 " .. target.name)

	blackboard.context.powered_move = false

	action:switch_motion_mode("attack")
	action:cruise_slow_scan()
	action:gimbal_scan(-math.pi / 2, math.pi / 2)

	request:sleep(30)
	action:dwell_scan(kStayDuration)

	blackboard.context.attacked_outpost = true
end

return intent
