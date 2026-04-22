---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local profile = require("competition_profile")
local navigate_to_task = require("task.navigate-to")
local follow_waypoints = require("task.follow-waypoints")
local cruise_loop = require("task.cruise-loop")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local function get_option(name, fallback)
	local value = rawget(option, name)
	if value == nil then
		return fallback
	end
	return value
end

local function as_number(value, fallback)
	local converted = tonumber(value)
	if converted == nil then
		return fallback
	end
	return converted
end

local function as_boolean(value, fallback)
	if type(value) == "boolean" then
		return value
	end

	if type(value) == "number" then
		return value ~= 0
	end

	if type(value) == "string" then
		local lowered = string.lower(value)
		if lowered == "true" or lowered == "1" then
			return true
		end
		if lowered == "false" or lowered == "0" then
			return false
		end
	end

	return fallback
end

local function normalize_stage(stage)
	if type(stage) ~= "string" then
		return "UNKNOWN"
	end
	return string.upper(stage)
end

local function stage_matches(stage, expected)
	local current = normalize_stage(stage)
	local target = normalize_stage(expected)
	if current == target then
		return true
	end
	if target == "START" then
		return current == "STARTED"
	end
	if target == "STARTED" then
		return current == "START"
	end
	return false
end

local function parse_number_list(raw)
	local numbers = {}
	local kind = type(raw)

	if kind == "table" then
		for _, value in ipairs(raw) do
			if type(value) == "number" then
				numbers[#numbers + 1] = value
			elseif type(value) == "string" then
				local converted = tonumber(value)
				if converted ~= nil then
					numbers[#numbers + 1] = converted
				end
			elseif type(value) == "table" then
				local x = tonumber(value.x or value[1])
				local y = tonumber(value.y or value[2])
				if x ~= nil and y ~= nil then
					numbers[#numbers + 1] = x
					numbers[#numbers + 1] = y
				end
			end
		end
	elseif kind == "string" then
		for token in string.gmatch(raw, "[-+]?%d*%.?%d+") do
			local converted = tonumber(token)
			if converted ~= nil then
				numbers[#numbers + 1] = converted
			end
		end
	end

	return numbers
end

local function parse_point(raw, fallback)
	if type(raw) == "table" and raw.x ~= nil and raw.y ~= nil then
		local x = tonumber(raw.x)
		local y = tonumber(raw.y)
		if x ~= nil and y ~= nil then
			return { x = x, y = y }
		end
	end

	local numbers = parse_number_list(raw)
	if #numbers >= 2 then
		return {
			x = numbers[1],
			y = numbers[2],
		}
	end
	return fallback
end

local function parse_points(raw)
	local points = {}

	if type(raw) == "table" and type(raw[1]) == "table" then
		for _, point_raw in ipairs(raw) do
			local point = parse_point(point_raw, nil)
			if point ~= nil then
				points[#points + 1] = point
			end
		end
		if #points > 0 then
			return points
		end
	end

	local numbers = parse_number_list(raw)
	local index = 1
	while index + 1 <= #numbers do
		points[#points + 1] = {
			x = numbers[index],
			y = numbers[index + 1],
		}
		index = index + 2
	end
	return points
end

local function format_point(point)
	return string.format("(%.2f, %.2f)", point.x, point.y)
end

local function format_points(points)
	if #points == 0 then
		return "[]"
	end

	local fragments = {}
	for index, point in ipairs(points) do
		fragments[index] = format_point(point)
	end
	return "[" .. table.concat(fragments, ", ") .. "]"
end

local config = {
	start_stage = "STARTED",
	health_limit = 0,
	reach_tolerance = 0.2,
	waypoint_timeout = 15.0,
	entry_path = {},
	cruise_points = {},
	home = { x = 0.0, y = 0.0 },
	nav = {
		global_map = "rmul",
		launch_livox = false,
		launch_odin1 = false,
		use_sim_time = false,
	},
}

-- 配置项（均来自 rmcs_navigation.ros__parameters）：
-- 包内默认值来自：src/lua/competition_profile.lua
-- competition_start_stage: string，默认 "STARTED"，兼容写成 "START"/"start"
-- competition_health_limit: number，血量低于该值后回原点
-- competition_entry_path: number[] 或 string，格式 [x1, y1, x2, y2, ...]
-- competition_cruise_points: number[] 或 string，格式 [x1, y1, x2, y2, ...]
-- competition_home: number[] 或 {x=..., y=...}，默认 [0.0, 0.0]
-- competition_reach_tolerance: number，到点容差，默认 0.2
-- competition_waypoint_timeout: number，单点等待超时（秒），默认 15.0
local function load_config()
	option:set_handler(function(error)
		action:fuck("while fetch option: " .. error)
	end)

	config.start_stage = tostring(get_option("competition_start_stage", profile.start_stage or "STARTED"))
	config.health_limit = as_number(
		get_option("competition_health_limit", get_option("health_limit", profile.health_limit or 0)),
		as_number(profile.health_limit, 0)
	)
	config.reach_tolerance = as_number(get_option("competition_reach_tolerance", 0.2), 0.2)
	config.waypoint_timeout = as_number(get_option("competition_waypoint_timeout", 15.0), 15.0)

	config.entry_path = parse_points(get_option("competition_entry_path", profile.entry_path or {}))
	config.cruise_points = parse_points(get_option("competition_cruise_points", profile.cruise_points or {}))
	config.home = parse_point(
		get_option("competition_home", profile.home or { 0.0, 0.0 }),
		parse_point(profile.home, { x = 0.0, y = 0.0 })
	)

	config.nav.global_map = tostring(get_option("global_map", profile.global_map or "training"))
	config.nav.launch_livox =
		as_boolean(get_option("launch_livox", profile.launch_livox), true)
	config.nav.launch_odin1 =
		as_boolean(get_option("launch_odin1", profile.launch_odin1), false)
	config.nav.use_sim_time =
		as_boolean(get_option("use_sim_time", profile.use_sim_time), false)

	blackboard.rule.health_limit = config.health_limit
end

local function is_stage_started()
	return stage_matches(blackboard.game.stage, config.start_stage)
end

local function is_low_health()
	return blackboard.user.health < config.health_limit
end

local function restart_navigation()
	action:restart_navigation({
		global_map = config.nav.global_map,
		launch_livox = config.nav.launch_livox,
		launch_odin1 = config.nav.launch_odin1,
		use_sim_time = config.nav.use_sim_time,
	})
end

local function navigate_to(point, args)
	local options = args or {}
	return navigate_to_task({
		target = point,
		reach_tolerance = config.reach_tolerance,
		waypoint_timeout = config.waypoint_timeout,
		is_stage_started = is_stage_started,
		is_low_health = is_low_health,
		interrupt_on_low_health = options.interrupt_on_low_health,
		on_timeout = function(target)
			action:warn("导航点超时: " .. format_point(target))
		end,
	})
end

local function make_reverse_path(points)
	local route = {}
	for index = #points, 1, -1 do
		local point = points[index]
		route[#route + 1] = { x = point.x, y = point.y }
	end
	return route
end

local function copy_point(point)
	return { x = point.x, y = point.y }
end

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)
	load_config()
	action:bind(scheduler)
	action:stop_navigation()

	if get_option("enable_goal_topic_forward", false) then
		action:switch_topic_forward(true)
	end

	action:info(ascii.banner)
	action:info("competition endpoint ready")
	action:info("start_stage=" .. config.start_stage)
	action:info("health_limit=" .. string.format("%.1f", config.health_limit))
	action:info("entry_path=" .. format_points(config.entry_path))
	action:info("cruise_points=" .. format_points(config.cruise_points))
	action:info("home=" .. format_point(config.home))

	scheduler:append_task(function()
		local State = {
			WAIT_START = "WAIT_START",
			ENTRY_PATH = "ENTRY_PATH",
			CRUISE = "CRUISE",
			RETURN_HOME = "RETURN_HOME",
		}
		local mission = fsm:new(State.WAIT_START)
		local navigation_online = false
		local reached_entry_path = {}
		local return_route = {}
		local return_route_announced_index = 0
		local return_route_finished = false
		local function reset_reached_entry_path()
			reached_entry_path = {}
		end
		local function record_reached_entry_path(point)
			reached_entry_path[#reached_entry_path + 1] = copy_point(point)
		end
		local function build_return_route_from_reached_entry_path()
			local route = make_reverse_path(reached_entry_path)
			route[#route + 1] = copy_point(config.home)
			return route
		end
		local function back_to_wait_start(handle, reason)
			if navigation_online then
				action:warn("导航停止: " .. reason)
				action:stop_navigation()
				action:update_chassis_vel(0.0, 0.0)
				navigation_online = false
			end
			handle:set_next(State.WAIT_START)
		end

		mission:use({
			state = State.WAIT_START,
			enter = function()
				reset_reached_entry_path()
				action:info("等待比赛开始，导航未启动")
			end,
			event = function(handle)
				if not is_stage_started() then
					request:sleep(0.1)
					return
				end

				if not navigation_online then
					restart_navigation()
					navigation_online = true
					action:info("比赛开始，导航已启动")
					request:sleep(1.0)
				end

				if is_low_health() then
					handle:set_next(State.RETURN_HOME)
					return
				end
				handle:set_next(State.ENTRY_PATH)
			end,
		})

		mission:use({
			state = State.ENTRY_PATH,
			enter = function()
				reset_reached_entry_path()
				if #config.entry_path == 0 then
					action:warn("entry_path 为空，直接进入巡航")
					return
				end
				action:info("开始执行过渡路径，共 " .. tostring(#config.entry_path) .. " 个点（重新记录）")
			end,
			event = function(handle)
				if not is_stage_started() then
					back_to_wait_start(handle, "比赛未开始或已结束")
					return
				end
				if is_low_health() then
					handle:set_next(State.RETURN_HOME)
					return
				end

				local success, reason = follow_waypoints({
					points = config.entry_path,
					navigate = navigate_to,
					interrupt_on_low_health = true,
					advance_on_timeout = true,
					retry_delay = 0.1,
					on_reached = function(point)
						record_reached_entry_path(point)
					end,
				})
				if reason == "stage_not_started" then
					back_to_wait_start(handle, "比赛阶段切回非 STARTED")
					return
				end
				if reason == "low_health" then
					handle:set_next(State.RETURN_HOME)
					return
				end
				if not success then
					request:sleep(0.1)
					return
				end

				handle:set_next(State.CRUISE)
			end,
		})

		mission:use({
			state = State.CRUISE,
			enter = function()
				if #config.cruise_points == 0 then
					action:warn("cruise_points 为空，将原地待命")
					return
				end
				action:info("进入巡航，共 " .. tostring(#config.cruise_points) .. " 个点循环")
			end,
			event = function(handle)
				if not is_stage_started() then
					back_to_wait_start(handle, "比赛未开始或已结束")
					return
				end
				if is_low_health() then
					handle:set_next(State.RETURN_HOME)
					return
				end

				if #config.cruise_points == 0 then
					request:sleep(0.5)
					return
				end

				local success, reason = cruise_loop({
					points = config.cruise_points,
					navigate = navigate_to,
					retry_delay = 0.1,
					idle_sleep = 0.5,
				})
				if reason == "stage_not_started" then
					back_to_wait_start(handle, "比赛阶段切回非 STARTED")
					return
				end
				if reason == "low_health" then
					handle:set_next(State.RETURN_HOME)
					return
				end
				if not success then
					request:sleep(0.1)
					return
				end
			end,
		})

		mission:use({
			state = State.RETURN_HOME,
			enter = function()
				return_route = build_return_route_from_reached_entry_path()
				return_route_announced_index = 0
				return_route_finished = false

				action:warn("血量低于阈值，沿已到达 entry_path 原路回家")
				action:info(
					string.format(
						"已记录 entry_path 点数: %d，回撤路径点数: %d",
						#reached_entry_path,
						#return_route
					)
				)
			end,
			event = function(handle)
				if not is_stage_started() then
					back_to_wait_start(handle, "比赛未开始或已结束")
					return
				end

				if not return_route_finished then
					local success, reason = follow_waypoints({
						points = return_route,
						navigate = navigate_to,
						interrupt_on_low_health = false,
						advance_on_timeout = false,
						retry_delay = 0.1,
						on_before_point = function(point, index, total)
							if return_route_announced_index ~= index then
								return_route_announced_index = index
								action:info(
									string.format("回撤目标(%d/%d): %s", index, total, format_point(point))
								)
							end
						end,
					})
					if reason == "stage_not_started" then
						back_to_wait_start(handle, "比赛阶段切回非 STARTED")
						return
					end
					if not success then
						request:sleep(0.1)
						return
					end
					return_route_finished = true
					action:info("已回到 home，等待补血")
				end

				action:navigate(config.home)
				request:wait_until({
					monitor = function()
						return blackboard.condition.near(config.home, config.reach_tolerance)
							or (not is_stage_started())
					end,
					timeout = config.waypoint_timeout,
				})
				if not is_stage_started() then
					back_to_wait_start(handle, "比赛未开始或已结束")
					return
				end

				-- Once we have safely returned home, resume mission automatically when health recovers.
				if not is_low_health() then
					action:info("血量恢复，重新出击")
					handle:set_next(State.ENTRY_PATH)
					return
				end
				request:sleep(0.2)
			end,
		})

		if not mission:init_ready(State) then
			error("比赛状态机初始化失败，有状态未注册")
		end

		while true do
			mission:spin_once()
			request:yield()
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function()
	action:stop_navigation()
end

--- 由 NAV2 发布的目标速度值，在此处理回调
on_control = function(vx, vy, _)
	action:update_chassis_vel(vx, vy)
end
