local function PointPair(points)
	return {
		ours = { x = points[1][1], y = points[1][2] },
		them = { x = points[2][1], y = points[2][2] },
	}
end

local function create_default_blackboard()
	local last_our_dart_nmber_of_hits = nil
	local result = {
		-- Dynamic Information
		user = {
			health = 0,
			bullet = 0,
			chassis_power_limit = 0,
			chassis_power = 0,
			chassis_buffer_energy = 0,
			chassis_output_status = false,
			shooter_cooling = 0,
			shooter_heat_limit = 0,
			bullet_42mm = 0,
			fortress_17mm_bullet = 0,
			initial_speed = 0,
			shoot_timestamp = 0,
			x = 0,
			y = 0,
			yaw = 0,

			mode = "UNKNOWN",
		},
		game = {
			stage = "UNKNOWN",
			
			outpost_health = 0, -- 前哨站血量
			base_health = 0, -- 基地血量

			hero_health = 150,
			infantry_1_health = 150,
			infantry_2_health = 150,
			engineer_health = 250,

			hero_position = { x = 0.0, y = 0.0 },
			infantry_1_position = { x = 0.0, y = 0.0 },
			infantry_2_position = { x = 0.0, y = 0.0 },
			engineer_position = { x = 0.0, y = 0.0 },

			remaining_time = 0, -- 比赛剩余时间
			gold_coin = 0, -- 队伍剩余金币数
			exchangeable_ammunition_quantity = 0, -- 队伍 17mm 允许发弹量的剩余可兑换数
			
			our_dart_nmber_of_hits = false, -- 己方飞镖击中次数
			fortress_occupied = false, -- 己方堡垒是否被占领
			big_energy_mechanism_activated = false, -- 大能量机关是否被激活
			small_energy_mechanism_activated = false, -- 小能量机关是否被激活
		},
		play = {
			rswitch = "UNKNOWN",
			lswitch = "UNKNOWN",
		},
		meta = {
			timestamp = 0, -- 秒
			fsm_state = "unknown",
			fsm_return_stage = "before_fluctuant",
		},
		referee = {
			sync_timestamp = 0,
			robot_id = 0,
			robots_hp = {
				ally_1 = 0,
				ally_2 = 0,
				ally_3 = 0,
				ally_4 = 0,
				reserved = 0,
				ally_7 = 0,
				outpost = 0,
				base = 0,
			},

			can_confirm_free_revive = false,
			can_exchange_instant_revive = false,
			instant_revive_cost = 0,
			exchanged_bullet = 0,
			remote_bullet_exchange_count = 0,
			sentry_mode = 0,
			energy_mechanism_activatable = false,

			red_score = 0,
			blue_score = 0,
		},

		-- Static Information
		rule = {
			decision = "auxiliary",

			-- 自身状态类规则

			health_limit = 210,
			health_ready = 400,
			bullet_limit = 40,
			bullet_ready = 160,
			mode = "movement",

			-- 其他状态类规则

			-- 比赛相关
			time_of_the_competition_red_line = 90, --比赛剩余时间红线

			-- 队伍资源相关
			exchangeable_ammunition_quantity_red_line = 1000, -- 队伍 17mm 允许发弹量的剩余可兑换数红线
			gold_coin_red_line = 400, -- 队伍剩余金币数红线

			-- 前哨站相关
			outpost_health_red_line = 1500,

			-- 基地相关
			base_health_red_line = 2000,

			-- 友方机器人相关
			hero_health_ready_red_line = 50,
			infantry_1_health_ready_red_line = 50,
			infantry_2_health_ready_red_line = 50,
			engineer_health_ready_red_line = 50,

			-- 坐标类规则
			-- 定义顺序：ours = 0，them = 1

			-- 普通地形坐标
			fortress = PointPair { { 0, 0 }, { 0, 0 } }, -- 堡垒
			resupply_zone = PointPair { { 0, 0 }, { 0, 0 } }, -- 补给点
			road_zone_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 公路区
			road_zone_final = PointPair { { 0, 0 }, { 0, 0 } },
			launch_ramp_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 飞坡
			launch_ramp_final = PointPair { { 0, 0 }, { 0, 0 } },
			outpost_resupply = PointPair { { 0, 0 }, { 0, 0 } }, -- 前哨站补给点
			assembly_zone = PointPair { { 0, 0 }, { 0, 0 } },
			central_highland_near_fluctuant_road = PointPair { { 0, 0 }, { 0, 0 } }, -- 中央高地靠近起伏路一侧
			central_highland_near_doghole =  PointPair { { 0, 0 }, { 0, 0 } }, -- 中央高地靠近狗洞一侧
			central_highland_gain_point = PointPair { { 0, 0 }, { 0, 0 } }, -- 中央高地增益点
			central_highland_near_two_steps_and_outpost = PointPair { { 0, 0 }, { 0, 0 } }, -- 中央高地靠近二级台阶（二级台阶增益点和前哨站中间）
			base_left_gain_point = PointPair { { 0, 0 }, { 0, 0 } }, -- 左侧基地增益点
			base_right_gain_point = PointPair { { 0, 0 }, { 0, 0 } }, -- 右侧基地增益点

			-- 特殊跨越地形坐标
			road_tunnel_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 公路隧道
			road_tunnel_final = PointPair { { 0, 0 }, { 0, 0 } },
			one_step_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 一级台阶高点
			one_step_final = PointPair { { 0, 0 }, { 0, 0 } }, -- 一级台阶低点
			two_step_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 二级台阶高点
			two_step_final = PointPair { { 0, 0 }, { 0, 0 } }, 
			fluctuant_road_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 起伏路段
			fluctuant_road_final = PointPair { { 0, 0 }, { 0, 0 } },
			common_elevated_ground_begin = PointPair { { 0, 0 }, { 0, 0 } }, -- 普通高地（飞坡起点那个高地）
			common_elevated_ground_final = PointPair { { 0, 0 }, { 0, 0 } },
		},
	}

	result.getter = {
		rswitch = function()
			return result.play.rswitch
		end,
	}

	result.condition = {
		low_health = function()
			return result.user.health < result.rule.health_limit
		end,
		low_bullet = function()
			return result.user.bullet < result.rule.bullet_limit
		end,
		health_ready = function()
			return result.user.health >= result.rule.health_ready
		end,
		bullet_ready = function()
			return result.user.bullet >= result.rule.bullet_ready
		end,

		base_in_danger = function ()
			return result.game.base_health <= result.rule.base_health_red_line
		end,

		oupost_survival = function ()
			return result.game.outpost_health > 0
		end,

		dart_hit_first_time = function ()
			local current = result.game.our_dart_nmber_of_hits

			if last_our_dart_nmber_of_hits == nil then
				last_our_dart_nmber_of_hits = current
				return false
			end

			local triggered = last_our_dart_nmber_of_hits == 0 and current == 1
			last_our_dart_nmber_of_hits = current
			return triggered
		end,

		fortress_occupied = function ()
			return result.game.fortress_occupied
		end,

		big_energy_mechanism_activated = function ()
			return result.game.big_energy_mechanism_activated
		end,

		small_energy_mechanism_activated = function ()
			return result.game.small_energy_mechanism_activated
		end,

		game_close_to_end = function ()
			return result.game.remaining_time <= result.rule.time_of_the_competition_red_line
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
