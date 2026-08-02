local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points
local kStayDuration = 30 -- 秒，到达哨站后停留时长
local intent = {}
function intent:loop()
    action:info("前往敌方前哨站")
    action:switch_motion_mode("attack")
    action:gimbal_scan(-math.pi / 3, math.pi / 3)
    local target = Points.kAttackOutpost

    -- 兜底任务：永不放弃。失败则返回上一个确认点，重新搜索重试

    while true do
        local failed = false
        for i, path in ipairs(Map:search(bb.context.current, target)) do
            action:info("Execute path task: " .. i .. " " .. path.begin_name .. " -> " .. path.final_name)
            if not path.run() then
                action:warn("路径任务失败，返回 " .. bb.context.current.name .. " 重试")
                failed = true
                break
            end
            bb.context.current = path.final_point -- 每条边成功后推进
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
    action:info("已到达 " .. target.name .. "，停留 " .. kStayDuration .. "s")

    request:sleep(kStayDuration)

    blackboard.context.attacked_outpost = true
end

return intent
