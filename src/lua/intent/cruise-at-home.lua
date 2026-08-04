local action = require("action")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local kRoute = {
	Points.kBehindOutpost,
	Points.kSelfStepBegin,
	Points.kSelfDoubleStepsBegin,
	Points.kSelfHighlandBegin,
	Points.kAttackRune,
	Points.kAttackOutpost,
	Points.kAttackRune,
	Points.kSelfHighlandBegin,
	Points.kSelfDoubleStepsBegin,
	Points.kSelfStepBegin,
}

local kCruiseInterval = 5 -- 秒，每个节点停留时长

local intent = {}

local function start_index()
	local current = bb.context.current
	if current ~= nil then
		for index, route_point in ipairs(kRoute) do
			if route_point == current then
				return index
			end
		end
	end

	local nearest_index, nearest_distance = 1, math.huge
	for index, route_point in ipairs(kRoute) do
		local dx = route_point.x - bb.user.x
		local dy = route_point.y - bb.user.y
		local distance = dx * dx + dy * dy
		if distance < nearest_distance then
			nearest_index, nearest_distance = index, distance
		end
	end
	return nearest_index
end

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
	action:info("在家中巡游，形如地刺")
	action:switch_motion_mode("attack")

	local count = #kRoute
	local index = start_index()
	local begin = index

	while true do
		local from = kRoute[index]
		local to = kRoute[index % count + 1]

		action:cruise_slow_scan()
		if cruise_leg(from, to) then
			action:cruise_fast_scan()
			action:dwell_scan(kCruiseInterval)
			index = index % count + 1
		else
			action:warn("巡游段重试: " .. from.name .. " -> " .. to.name)
			action:cruise_slow_scan()
			action:navigate(from)
			request:wait_until {
				monitor = function()
					return bb.condition.near(from, 0.5)
				end,
				timeout = 10,
			}
		end

		if index == begin then
			action:info("完成一圈家中巡游")
		end
	end
end

return intent
