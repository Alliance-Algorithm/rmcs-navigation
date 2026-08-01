GameStage = {
	NOT_START = "NOT_START",
	PREPARATION = "PREPARATION",
	REFEREE_CHECK = "REFEREE_CHECK",
	COUNTDOWN = "COUNTDOWN",
	STARTED = "STARTED",
	SETTLING = "SETTLING",
	UNKNOWN = "UNKNOWN",
}
SentryEvent = {
	SWITCH_POSE_ATTACK = "SWITCH_POSE_ATTACK",
	SWITCH_POSE_DEFENSE = "SWITCH_POSE_DEFENSE",
	SWITCH_POSE_MOVE = "SWITCH_POSE_MOVE",
	SWITCH_POSE_POWERED_ATTACK = "SWITCH_POSE_POWERED_ATTACK",
	SWITCH_POSE_POWERED_DEFENSE = "SWITCH_POSE_POWERED_DEFENSE",
	SWITCH_POSE_POWERED_MOVE = "SWITCH_POSE_POWERED_MOVE",
	CONFIRM_REBIRTH = "CONFIRM_REBIRTH",
	CONFIRM_INSTANT_REBIRTH = "CONFIRM_INSTANT_REBIRTH",
	EXCHANGE_AMMO_SUPPLY_POINT = "EXCHANGE_AMMO_SUPPLY_POINT",
	EXCHANGE_AMMO_REMOTE = "EXCHANGE_AMMO_REMOTE",
	EXCHANGE_HP_REMOTE = "EXCHANGE_HP_REMOTE",
	ACTIVATE_ENERGY_CORE = "ACTIVATE_ENERGY_CORE",
}

local function create_default_blackboard()
	local result = {
		-- Dynamic Information
		user = {
			health = 400,
			bullet = 300,
			chassis_power_limit = 100,
			x = 0,
			y = 0,
			yaw = 0,
		},
		game = {
			stage = GameStage.UNKNOWN,
			enemy_outpost_hp = 0,
			enemy_base_hp = 0,
		},
		play = {
			rswitch = "UNKNOWN",
			lswitch = "UNKNOWN",
		},
		autoaim = {
			should_control = false,
		},
		meta = {
			timestamp = 0, -- 秒
		},

		-- Static Information
		rule = {
			-- 状态类规则
			health_limit = 200,
			health_critical = 50,
			bullet_limit = 20,
		},

		-- context
		context = {
			current = nil, -- 当前地图节点，由 Map 边 Task 成功后推进

			power_move = false,
		},
	}

	result.getter = {
		rswitch = function()
			return result.play.rswitch
		end,
		stage = function()
			return result.game.stage
		end,
	}

	result.condition = {
		low_health = function()
			return result.user.health < result.rule.health_limit
		end,
		low_bullet = function()
			return result.user.bullet < result.rule.bullet_limit
		end,

		--- @param target {x: number, y: number}
		--- @param tolerance? number|{x: number, y: number}
		near = function(target, tolerance)
			local x_diff = math.abs(target.x - result.user.x)
			local y_diff = math.abs(target.y - result.user.y)

			if type(tolerance) == "number" then
				return x_diff <= tolerance and y_diff <= tolerance
			end

			local limit = tolerance or { x = 0.05, y = 0.05 }
			return x_diff <= limit.x and y_diff <= limit.y
		end,
	}

	return result
end

local blackboard_singleton = create_default_blackboard()

local BlackboardDetails = {}
function BlackboardDetails.singleton()
	return blackboard_singleton
end

return BlackboardDetails
