local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()

local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kCruiseInterval = 3 -- 秒，每个节点停留时长

local them_home_points = {
	Points.kThemStepBegin,
	Points.kThemDoubleStepsBegin,
	Points.kThemHighlandBegin,
    Points.kThemFortressRight,
}

local intent = {}

function intent:loop()
	action:info("在对方半场巡游，四个点每 " .. kCruiseInterval .. "s 切换一次")
	action:switch_motion_mode("attack")
	action:gimbal_scan(0, 0)

	local index = 1

	while true do
		local target = them_home_points[index]

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

		action:info("到达 " .. target.name .. "，停留 " .. kCruiseInterval .. "s")
		request:sleep(kCruiseInterval)

		index = index % #them_home_points + 1
	end
end

return intent