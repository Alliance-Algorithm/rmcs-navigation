local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local intent = {}

function intent:loop()
    action:info("前往敌方基地前方")
    action:switch_motion_mode("attack")
    action:cruise_slow_scan()

    local target = Points.kAttackBaseFront
    while not action:follow_path(Map, target) do
        action:navigate_until(bb.context.current, 0.5, 10)
    end
    action:info("已到达 " .. target.name)

    action:switch_motion_mode("attack")
    action:cruise_fast_scan()
    request:sleep(300) -- 基本等于永久停留
end

return intent
