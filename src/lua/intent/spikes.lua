local action = require("action")
local bb = require("blackboard").singleton()
local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	local points = bb.rule.spike_points
	local index = 1

	action:info("开启地刺意图，随机收取过路费")
	while true do
		local point = points[index]
		index = index + 1
		if index > #points then
			index = 1
		end

		action:navigate { x = point[1], y = point[2] }
		action:switch_motion_mode("attack")
		action:gimbal_scan(0, 0)

		request:sleep(bb.rule.spike_interval)
	end
end

return intent
