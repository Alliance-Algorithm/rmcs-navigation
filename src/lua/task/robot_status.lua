local action = require("action")
local clock = require("util.clock")
local request = require("util.scheduler").request

return function()
	local last_attack_timestamp = clock:now()
	local last_hp = 0

	while true do
		local hp = blackboard.user.health
		if last_hp > hp then
			action:info("Our robot(" .. hp .. ") is under attack")
			last_attack_timestamp = clock:now()
		end
		last_hp = hp

		local under_attack = clock:now() - last_attack_timestamp < 5
		action:update_under_attack(under_attack)

		if blackboard.autoaim.should_control then
			action:gimbal_suspend_during(2)
		end

		request:yield()
	end
end
