local action = require("action")
local api = require("api")
local clock = require("util.clock")
local request = require("util.scheduler").request

return function()
	local last_attack_timestamp = clock:now()
	local last_attack_seen = clock:now()
	local last_hp = 0

	local last_posture = nil
	local last_sent = clock:now()
	local last_attack_base_seen = nil
	local last_powered_attack_seen = 0
	local kPoseEvent = {
		POWERED_DEFENSE = SentryEvent.SWITCH_POSE_POWERED_DEFENSE,
		DEFENSE = SentryEvent.SWITCH_POSE_DEFENSE,
		POWERED_MOVE = SentryEvent.SWITCH_POSE_MOVE,
		ATTACK = SentryEvent.SWITCH_POSE_ATTACK,
		POWERED_ATTACK = SentryEvent.SWITCH_POSE_POWERED_ATTACK,
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

		local power_defense = hp < blackboard.rule.health_critical
		local defense = hp < blackboard.rule.health_limit or (climbing and not action.climb.up)
		local powered_move = blackboard.context.powered_move
		local attack = blackboard.autoaim.should_control
		local attack_base = attack and blackboard.context.runing_intent == "attack-base"
		if attack then
			last_attack_seen = clock:now()
		end
		if attack_base then
			if last_attack_base_seen == nil then
				last_attack_base_seen = clock:now()
			end
		else
			last_attack_base_seen = nil
		end
		local powered_attack = attack_base and clock:now() - last_attack_base_seen >= 5
		if powered_attack then
			last_powered_attack_seen = clock:now()
		end

		local desired
		if powered_attack then
			desired = "POWERED_ATTACK"
		elseif
			not attack_base
			and last_posture == "POWERED_ATTACK"
			and clock:now() - last_powered_attack_seen < 2
		then
			desired = "POWERED_ATTACK"
		elseif attack_base then
			desired = "ATTACK"
		elseif power_defense then
			desired = "POWERED_DEFENSE"
		elseif defense then
			desired = "DEFENSE"
		elseif attack then
			desired = "ATTACK"
		elseif last_posture == "ATTACK" and clock:now() - last_attack_seen < 2 then
			desired = "ATTACK"
		elseif powered_move then
			desired = "POWERED_MOVE"
		else
			desired = "DEFENSE"
		end

		if desired ~= last_posture then
			last_posture = desired
			last_sent = clock:now()
			action:info("Sentry pose -> " .. desired)
			api.sentry_event(kPoseEvent[desired])
		elseif clock:now() - last_sent >= 6 then
			last_sent = clock:now()
			action:info("Sentry pose resend -> " .. desired)
			api.sentry_event(kPoseEvent[desired])
		end

		request:yield()
	end
end
