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

local function PointPair(points)
	return {
		ours = { x = points[1][1], y = points[1][2] },
		them = { x = points[2][1], y = points[2][2] },
	}
end

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
		},
		play = {
			rswitch = "UNKNOWN",
			lswitch = "UNKNOWN",
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

			-- 坐标类规则
			-- 定义顺序：ours = 0，them = 1

			-- 普通地形坐标
			home = { x = -2, y = -4.8 },
			-- home = { x = 0, y = 0 },

			fortress = PointPair { { 0, 0 }, { 0, 0 } }, -- 堡垒
			resupply_zone = PointPair { { 0, 0 }, { 0, 0 } }, -- 补给点
			road_zone_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 公路区
			road_zone_final = PointPair { { 0, 0 }, { 0, 0 } },
			launch_ramp_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 飞坡
			launch_ramp_final = PointPair { { 0, 0 }, { 0, 0 } },
			outpost_resupply = PointPair { { 0, 0 }, { 0, 0 } }, -- 前哨站补给点
			assembly_zone = PointPair { { 0, 0 }, { 0, 0 } },

			-- 特殊跨越地形坐标
			road_tunnel_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 公路隧道
			road_tunnel_final = PointPair { { 0, 0 }, { 0, 0 } },
			one_step_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 一级台阶
			one_step_final = PointPair { { 0, 0 }, { 0, 0 } },
			two_step_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 二级台阶
			two_step_final = PointPair { { 0, 0 }, { 0, 0 } },
			common_elevated_ground_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 普通高地（飞坡起点那个高地）
			common_elevated_ground_final = PointPair { { 0, 0 }, { 0, 0 } },

			-- 地刺巡逻点
			spike_points = {
				{ x = 5.4, y = -2.6 },
				{ x = 6.3, y = -3.7 },
				-- { 1.0, 0.0 },
				-- { 2.0, 0.0 },
			},
			spike_interval = 8,

			outpost_attack_point = { x = 7.0, y = 4.8 },
		},

		-- context
		context = {
			last_intent = Intent.nothing,
			hint_intent = Intent.nothing,

			intent_to_return = Intent.nothing,
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
