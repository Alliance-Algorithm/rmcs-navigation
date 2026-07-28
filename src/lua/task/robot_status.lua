local action = require("action")
local api = require("api")
local clock = require("util.clock")
local request = require("util.scheduler").request

return function()
	local last_attack_timestamp = clock:now()
	local last_hp = 0

	local last_posture = nil
	local kPoseEvent = {
		POWERED_DEFENSE = SentryEvent.SWITCH_POSE_POWERED_DEFENSE,
		DEFENSE = SentryEvent.SWITCH_POSE_DEFENSE,
		POWERED_MOVE = SentryEvent.SWITCH_POSE_POWERED_MOVE,
		ATTACK = SentryEvent.SWITCH_POSE_ATTACK,
	}

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
			action:gimbal_suspend_during(1)
		end

		local s = api.get_climb_status()
		local climbing = s > 0 and s < 1

		local power_defense = hp < blackboard.rule.health_critical or (climbing and action.climb.up)
		local defense = hp < blackboard.rule.health_limit or (climbing and not action.climb.up)
		local power_move = blackboard.context.power_move
		local attack = blackboard.autoaim.should_control

		local desired
		if power_defense then
			desired = "POWERED_DEFENSE"
		elseif defense then
			desired = "DEFENSE"
		elseif power_move then
			desired = "POWERED_MOVE"
		elseif attack then
			desired = "ATTACK"
		else
			desired = "DEFENSE"
		end

		if desired ~= last_posture then
			last_posture = desired
			action:info("Sentry pose -> " .. desired)
			api.sentry_event(kPoseEvent[desired])
		end

		request:yield()
	end
end
