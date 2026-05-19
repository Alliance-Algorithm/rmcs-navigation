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
			bullet = 100,
			chassis_power_limit = 100,
			x = 0,
			y = 0,
			yaw = 0,
		},
		game = {
			stage = "UNKNOWN",
		},
		outpost = {
			ours = { health = 1500 },
			them = { health = 1500 },
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
			decision = "auxiliary",

			-- 状态类规则

			health_limit = 60,
			health_ready = 400,
			bullet_limit = 400,
			bullet_ready = 300,

			-- 坐标类规则
			-- 定义顺序：ours = 0，them = 1

			-- 普通地形坐标
			fortress = PointPair { { 7.2, 8.1 }, { 22.8, 7.9 } },          -- 堡垒
			resupply_zone = PointPair { { 3.0, 2.0 }, { 26.0, 14.0 } },     -- 补给点
			road_zone_begin = PointPair { { 8.2, 3.7 }, { 20.8, 12.3 } }, -- 公路区
			road_zone_final = PointPair { { 11.0, 1.4 }, { 18.0, 14.6 } },
			launch_ramp_begin = PointPair { { 12.7, 1.1 }, { 16.3, 14.9 } }, -- 飞坡
			launch_ramp_final = PointPair { { 15.5, 1.1 }, { 13.5, 14.9 } },
			outpost_resupply = PointPair { { 12.0, 4.3 }, { 17.0, 11.7 } }, -- 前哨站补给点
			assembly_zone = PointPair { { 13.5, 8.7 }, { 15.5, 7.3 } },
			
			-- 特殊跨越地形坐标
			road_tunnel_begin = PointPair { { 0, 0 }, { 0, 0 } },   -- 公路隧道
			road_tunnel_final = PointPair { { 0, 0 }, { 0, 0 } },
			one_step_begin = PointPair { { 8.7, 2.1 }, { 20.3, 26.9 } },      -- 一级台阶
			one_step_final = PointPair { { 8.7, 3.4 }, { 20.3, 25.6 } },
			two_step_begin = PointPair { { 0, 0 }, { 0, 0 } },      -- 二级台阶
			two_step_final = PointPair { { 0, 0 }, { 0, 0 } },
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
		getout = function()
			return result.user.health >= result.rule.health_ready and result.user.bullet > 100
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
