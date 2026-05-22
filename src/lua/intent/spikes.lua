local action = require("action")
local bb = require("blackboard").singleton()
local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	local spike_points = bb.rule.spike_points

	local spike_1 = spike_points[1]
	local spike_2 = spike_points[2]

	local outpost = bb.rule.outpost_attack_point

	action:gimbal_toward(0, 0)
	action:navigate(spike_1)
	request:sleep(2)

	action:switch_motion_mode("attack")
	action:info("开启地刺意图，随机收取过路费")
	while true do
		action:switch_motion_mode("attack")
		action:reset_gimbal_speed()

		action:navigate(spike_1)
		action:gimbal_scan(0, 0)
		request:sleep(10)

		action:navigate(spike_2)
		action:gimbal_scan(0, 0)
		request:sleep(10)

		action:navigate(outpost)
		action:gimbal_scan(2 * math.pi - 1.6, 0.5)
		action:set_gimbal_pt(4)
		request:sleep(20)
	end
end

return intent
