GameStage = {
	NOT_START = "NOT_START",
	PREPARATION = "PREPARATION",
	REFEREE_CHECK = "REFEREE_CHECK",
	COUNTDOWN = "COUNTDOWN",
	STARTED = "STARTED",
	SETTLING = "SETTLING",
	UNKNOWN = "UNKNOWN",
}
Intent = {
	nothing = "nothing",
	spikes = "spikes",
	supply = "supply",
}
SentryEvent = {
	SWITCH_POSE_ATTACK = 0,
	SWITCH_POSE_DEFENSE = 1,
	SWITCH_POSE_MOVE = 2,
	SWITCH_POSE_POWERED_ATTACK = 3,
	SWITCH_POSE_POWERED_DEFENSE = 4,
	SWITCH_POSE_POWERED_MOVE = 5,
	CONFIRM_REBIRTH = 6,
	CONFIRM_INSTANT_REBIRTH = 7,
	EXCHANGE_AMMO_SUPPLY_POINT = 8,
	EXCHANGE_AMMO_REMOTE = 9,
	EXCHANGE_HP_REMOTE = 10,
	ACTIVATE_ENERGY_CORE = 11,
}

local function create_default_blackboard()
	local result = {
		-- Dynamic Information
		user = {
			health = 400,
			bullet = 300,
			chassis_power_limit = 100,
			should_shoot = false,
			enemy_visible = false,
			motion_mode = "normal",
			x = 0,
			y = 0,
			yaw = 0,
		},
		game = {
			stage = GameStage.UNKNOWN,
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
			bullet_limit = 20,
			health_ready = 350,
			bullet_ready = 95,
			supply_interval = 6, -- 返回补给区，只补充对应的秒数，防止因意外而阻塞其他机器人进出
		},

		-- context
		context = {
			last_intent = Intent.nothing,
			hint_intent = Intent.nothing,

			intent_to_return = Intent.nothing,

			current = nil, -- 当前地图节点，由 Map 边 Task 成功后推进
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
				return math.sqrt(x_diff * x_diff + y_diff * y_diff) <= tolerance
			end

			local limit = tolerance or { x = 0.05, y = 0.05 }
			return math.sqrt(x_diff * x_diff + y_diff * y_diff) <= math.sqrt(limit.x * limit.x + limit.y * limit.y)
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
