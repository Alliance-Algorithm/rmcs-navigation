local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()
local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points
local kCruiseInterval = 5   -- 秒，每个巡逻点停留时长
local kNavScanSpeed = 3.2   -- 秒/弧度，导航中云台慢扫（约 20s 一圈）
local kDwellScanSpeed = 1.5 -- 秒/弧度，停留时云台快扫（约 10.7s 一圈）
local patrol_points = {
    Points.kNearThemOutpost,
    Points.kThemDoubleStepsFinal,
}

local intent = {}
function intent:loop()
    action:info("在对方前哨与双台阶终点之间巡逻，每 " .. kCruiseInterval .. "s 切换一次")
    action:switch_motion_mode("attack")
    local index = 1
    while true do
        local target = patrol_points[index]
        -- 兜底任务：永不放弃。失败则返回上一个确认点，重新搜索重试
        while true do
            local failed = false
            action:gimbal_scan(0, 0, kNavScanSpeed)
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
            action:gimbal_scan(0, 0, kNavScanSpeed)
            action:navigate(bb.context.current)
            request:wait_until {
                monitor = function()
                    return bb.condition.near(bb.context.current, 0.5)
                end,
                timeout = 20,
            }
        end
        action:info("到达 " .. target.name .. "，停留 " .. kCruiseInterval .. "s")
        action:gimbal_scan(0, 0, kDwellScanSpeed)
        request:sleep(kCruiseInterval)
        index = index % #patrol_points + 1
    end
end

return intent
