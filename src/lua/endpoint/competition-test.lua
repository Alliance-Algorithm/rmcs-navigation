---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local ReturnStage = require("util.return-stage")

local ForwardPressIntent = require("intent.forward-press")
local GuardHomeIntent = require("intent.guard-home")
local KeepCruiseIntent = require("intent.keep-cruise")
local StartCruiseIntent = require("intent.start-cruise")
local escape_to_home = require("intent.escape-to-home")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local edges = require("util.edge").new()

---
--- Export Context
---

blackboard = require("blackboard").singleton()

local runtime = {
	ours_zone = nil,
	switch_interval = nil,
	return_stage = nil,
	escape_route = nil,
	forward_press_mode = nil,
	forward_press_started_at = nil,
	current_state = "idle",
	current_phase = "none",
	current_intent = nil,
	navigation_ready = false,
}

local forward_press_duration = 30.0

local requests = {
	start = false,
}

local job = {
	handle = nil,
	name = nil,
	done = false,
	success = false,
}

local function read_option(name, fallback)
	local value = option[name]
	if value == nil then
		return fallback
	end
	return value
end

local function configure_test_rule()
	local rule = blackboard.rule

	rule.health_limit = read_option("health_limit", 210)
	rule.health_ready = read_option("health_ready", 400)
	rule.bullet_limit = read_option("bullet_limit", 40)
	rule.bullet_ready = read_option("bullet_ready", 300)
	rule.time_of_the_competition_red_line = read_option("time_of_the_competition_red_line", 90)
	rule.exchangeable_ammunition_quantity_red_line =
		read_option("exchangeable_ammunition_quantity_red_line", 1000)
	rule.gold_coin_red_line = read_option("gold_coin_red_line", 400)
	rule.outpost_health_red_line = read_option("outpost_health_red_line", 1500)
	rule.base_health_red_line = read_option("base_health_red_line", 2000)
	rule.hero_health_ready_red_line = read_option("hero_health_ready_red_line", 50)
	rule.infantry_1_health_ready_red_line =
		read_option("infantry_1_health_ready_red_line", 50)
	rule.infantry_2_health_ready_red_line =
		read_option("infantry_2_health_ready_red_line", 50)
	rule.engineer_health_ready_red_line =
		read_option("engineer_health_ready_red_line", 50)

	-- Ours side sample points
	-- 暂时全为0 
	rule.resupply_zone.ours = { x = 0.0, y = 0.0 }   		                    -- 家
	rule.fluctuant_road_begin.ours = { x = 0.0, y = 0.0 }		                -- 起伏路段起点
	rule.fluctuant_road_final.ours = { x = 0.0, y = 0.0 }		                -- 起伏路段终点
	rule.one_step_begin.ours = { x = 0.0, y = 0.0 }			                    -- 一级台阶高点（先随便标个回家路上的点）
	rule.one_step_final.ours = { x = 0.0, y = 0.0 }			                    -- 一级台阶低点（先随便标个回家路上的点）
	rule.central_highland_near_fluctuant_road.ours = { x = 0.0, y = 0.0 }  	    -- 高地靠近起伏路
	rule.central_highland_near_doghole.ours = { x = 0.0, y = 0.0 }			    -- 高地靠近狗洞
    rule.base_left_gain_point.ours = { x = 0.0, y = 0.0 }                       -- 左侧基地增益点
	rule.base_right_gain_point.ours = { x = 0.0, y = 0.0 }                      -- 右侧基地增益点
    rule.fortress.ours = { x = 0.0, y = 0.0 }                                   -- 堡垒增益点

	rule.central_highland_near_fluctuant_road.them = { x = 0.0, y = 0.0 }  	    -- 高地靠近起伏路
	rule.central_highland_near_doghole.them = { x = 0.0, y = 0.0 }			    -- 高地靠近狗洞
	rule.fluctuant_road_final.them = { x = 0.0, y = 0.0 }		                -- 起伏路段终点

end

local function reset_job_status()
	job.done = false
	job.success = false
end

local function cancel_job()
	if job.handle ~= nil then
		job.handle.cancel()
		job.handle = nil
	end
	job.name = nil
	reset_job_status()
end

local function run_job(name, fn)
	cancel_job()
	job.name = name
	reset_job_status()

	job.handle = scheduler:append_task(function()
		local ok, result = xpcall(fn, debug.traceback)
		job.handle = nil
		job.name = nil
		job.done = true

		if not ok then
			job.success = false
			action:fuck(string.format("fsm job '%s' failed:\n%s", name, result))
			return
		end

		job.success = (result ~= false)
		if not job.success then
			action:warn(string.format("fsm job '%s' finished with false", name))
		end
	end)
end

local function take_request(name)
	local value = requests[name]
	requests[name] = false
	return value
end

local function set_state(name)
	runtime.current_state = name
	blackboard.meta.fsm_state = name
	action:info("fsm state -> " .. name)
end

local function set_phase(name)
	runtime.current_phase = name
	action:info("fsm phase -> " .. name)
end

local function set_return_stage(name)
	assert(type(name) == "string", "return stage should be a string")

	if runtime.return_stage == name then
		return
	end

	runtime.return_stage = name
	blackboard.meta.fsm_return_stage = name
	action:info("fsm return-stage -> " .. name)
end

local function resolve_escape_route()
	assert(type(runtime.return_stage) == "string", "return_stage should be set before escape")
	return ReturnStage.resolve_escape_route(runtime.return_stage)
end

local function sync_intent_phase(intent)
	assert(intent ~= nil, "intent should exist before syncing phase")

	if type(intent.phase_name) == "function" then
		set_phase(intent:phase_name())
		return
	end

	set_phase("none")
end

local function sync_intent_return_stage(intent, apply_job_result)
	assert(intent ~= nil, "intent should exist before syncing return stage")
	assert(type(intent.return_stage) == "function", "intent should expose return_stage()")

	if apply_job_result and job.done and job.success and type(intent.on_job_succeeded) == "function" then
		intent:on_job_succeeded()
	end

	set_return_stage(intent:return_stage())
end

local function sync_intent_runtime(intent)
	sync_intent_phase(intent)
	sync_intent_return_stage(intent, false)
end

local function clear_forward_press_runtime()
	runtime.forward_press_mode = nil
	runtime.forward_press_started_at = nil
end

local function start_navigation()
	local ok, message = action:restart_navigation({
		global_map = read_option("global_map", "train_map"),
		launch_livox = read_option("launch_livox", true),
		launch_odin1 = read_option("launch_odin1", false),
		use_sim_time = read_option("use_sim_time", false),
	})
	if not ok then
		action:fuck("restart_navigation 触发失败: " .. tostring(message))
	end

	return ok, message
end

local function setup_edges()
	edges:on(blackboard.getter.rswitch, "UP", function()
		requests.start = true
	end)
end

local function create_intent_fsm()
	local State = {
		idle = "idle",
		start_cruise = "start_cruise",
		keep_cruise = "keep_cruise",
		guard_home = "guard_home",
		forward_press = "forward_press",
		escape = "escape",
		recover = "recover",
	}

	local condition = blackboard.condition
	local last_double_damage_activated = false
	local last_big_energy_mechanism_activated = false
	local last_small_energy_mechanism_activated = false
	local intent_fsm = fsm:new(State.idle)
	local function run_escape_job()
		assert(type(runtime.escape_route) == "string", "escape_route should be set before escape")
		run_job("escape_to_home", function()
			return escape_to_home(runtime.escape_route)
		end)
	end

	local function create_start_cruise_intent()
		runtime.current_intent = StartCruiseIntent.new({
			ours_zone = runtime.ours_zone,
		})
		sync_intent_runtime(runtime.current_intent)
	end
	local function create_keep_cruise_intent()
		runtime.current_intent = KeepCruiseIntent.new({
			ours_zone = runtime.ours_zone,
			switch_interval = runtime.switch_interval,
		})
		sync_intent_runtime(runtime.current_intent)
	end
	local function create_guard_home_intent(phase)
		runtime.current_intent = GuardHomeIntent.new({
			phase = phase,
			return_stage = runtime.return_stage,
		})
		sync_intent_runtime(runtime.current_intent)
	end
	local function create_forward_press_intent(mode)
		runtime.current_intent = ForwardPressIntent.new({
			mode = mode,
			switch_interval = runtime.switch_interval,
		})
		sync_intent_runtime(runtime.current_intent)
	end
	local function run_current_intent_job()
		assert(runtime.current_intent ~= nil, "current intent should exist before running intent job")
		sync_intent_runtime(runtime.current_intent)
		runtime.current_intent:run(run_job)
	end
	local function select_forward_press_mode()
		local dart_hit_first_time = condition.dart_hit_first_time()
		local double_damage_activated = condition.double_damage_activated()
		local big_energy_mechanism_activated = condition.big_energy_mechanism_activated()
		local small_energy_mechanism_activated = condition.small_energy_mechanism_activated()
		local double_damage_rising = double_damage_activated and not last_double_damage_activated
		local big_energy_rising =
			big_energy_mechanism_activated and not last_big_energy_mechanism_activated
		local small_energy_rising =
			small_energy_mechanism_activated and not last_small_energy_mechanism_activated

		last_double_damage_activated = double_damage_activated
		last_big_energy_mechanism_activated = big_energy_mechanism_activated
		last_small_energy_mechanism_activated = small_energy_mechanism_activated

		if small_energy_rising then
			return "two_step"
		end

		if dart_hit_first_time or double_damage_rising or big_energy_rising then
			return "one_step"
		end

		return nil
	end
	local function should_enter_guard_home()
		return condition.game_close_to_end() or condition.base_in_danger()
	end
	local function select_guard_home_phase()
		if condition.fortress_occupied() then
			return "cruise_in_front_of_base"
		end

		return "occupy_fortress"
	end
	local function advance_current_intent()
		assert(runtime.current_intent ~= nil, "current intent should exist before advancing")

		if type(runtime.current_intent.advance) ~= "function" then
			return false
		end

		return runtime.current_intent:advance()
	end
	local function enter_escape(handle)
		if runtime.current_intent ~= nil then
			sync_intent_return_stage(runtime.current_intent, true)
		end
		local route = resolve_escape_route()
		runtime.escape_route = route
		cancel_job()
		handle:set_next(State.escape)
	end

	intent_fsm:use({
		state = State.idle,
		enter = function()
			cancel_job()
			set_return_stage(ReturnStage.before_fluctuant)
			runtime.escape_route = nil
			runtime.current_intent = nil
			clear_forward_press_runtime()
			set_state(State.idle)
			set_phase("none")
			runtime.navigation_ready = false
		end,
		event = function(handle)
			if take_request("start") then
				local ok = start_navigation()
				runtime.navigation_ready = ok
			end

			if runtime.navigation_ready and blackboard.game.stage == "STARTED" then
				if should_enter_guard_home() then
					handle:set_next(State.guard_home)
				else
					handle:set_next(State.start_cruise)
				end
			end
		end,
	})

	intent_fsm:use({
		state = State.start_cruise,
		enter = function()
			set_state(State.start_cruise)
			set_return_stage(ReturnStage.before_fluctuant)
			create_start_cruise_intent()
			run_current_intent_job()
		end,
		event = function(handle)
			sync_intent_return_stage(runtime.current_intent, true)

			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				if advance_current_intent() then
					run_current_intent_job()
				else
					sync_intent_return_stage(runtime.current_intent, true)
					handle:set_next(State.keep_cruise)
				end
				return
			end

			action:warn(string.format(
				"fsm(start_cruise:%s): 导航失败，重试当前阶段",
				runtime.current_phase
			))
			run_current_intent_job()
		end,
	})

	intent_fsm:use({
		state = State.keep_cruise,
		enter = function()
			set_state(State.keep_cruise)
			set_return_stage(ReturnStage.after_fluctuant)
			clear_forward_press_runtime()
			create_keep_cruise_intent()
			run_current_intent_job()
		end,
		event = function(handle)
			sync_intent_return_stage(runtime.current_intent, true)

			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
				return
			end

			if should_enter_guard_home() then
				handle:set_next(State.guard_home)
				return
			end

			local forward_press_mode = select_forward_press_mode()
			if forward_press_mode ~= nil then
				runtime.forward_press_mode = forward_press_mode
				handle:set_next(State.forward_press)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				return
			end

			action:warn("fsm(keep_cruise): 导航失败，重试当前状态")
			run_current_intent_job()
		end,
	})

	intent_fsm:use({
		state = State.guard_home,
		enter = function()
			set_state(State.guard_home)
			clear_forward_press_runtime()
			create_guard_home_intent(select_guard_home_phase())
			run_current_intent_job()
		end,
		event = function(handle)
			sync_intent_return_stage(runtime.current_intent, true)

			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
				return
			end

			local next_phase = select_guard_home_phase()
			if runtime.current_intent:phase_name() ~= next_phase then
				action:info(string.format(
					"fsm(guard_home:%s): 切换到 %s",
					runtime.current_phase,
					next_phase
				))
				create_guard_home_intent(next_phase)
				run_current_intent_job()
				return
			end

			if not job.done then
				return
			end

			if job.success then
				if advance_current_intent() then
					run_current_intent_job()
					return
				end

				return
			end

			action:warn(string.format(
				"fsm(guard_home:%s): 导航失败，重试当前阶段",
				runtime.current_phase
			))
			run_current_intent_job()
		end,
	})

	intent_fsm:use({
		state = State.forward_press,
		enter = function()
			assert(type(runtime.forward_press_mode) == "string", "forward_press_mode should be set")
			set_state(State.forward_press)
			set_return_stage(ReturnStage.after_fluctuant)
			runtime.forward_press_started_at = clock:now()
			create_forward_press_intent(runtime.forward_press_mode)
			run_current_intent_job()
		end,
		event = function(handle)
			sync_intent_return_stage(runtime.current_intent, true)

			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
				return
			end

			if should_enter_guard_home() then
				handle:set_next(State.guard_home)
				return
			end

			local elapsed = clock:now() - runtime.forward_press_started_at
			if elapsed >= forward_press_duration then
				action:info(string.format(
					"fsm(forward_press:%s): 前压持续 %.1fs，返回 keep_cruise",
					runtime.current_phase,
					forward_press_duration
				))
				cancel_job()
				runtime.current_intent = nil
				handle:set_next(State.keep_cruise)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				return
			end

			action:warn(string.format(
				"fsm(forward_press:%s): 前压失败，重试当前状态",
				runtime.current_phase
			))
			run_current_intent_job()
		end,
	})

	intent_fsm:use({
		state = State.escape,
		enter = function()
			clear_forward_press_runtime()
			set_state(State.escape)
			set_phase("none")
			run_escape_job()
		end,
		event = function(handle)
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.recover)
				return
			end

			action:warn("fsm(escape): 导航失败，重试当前状态")
			run_escape_job()
		end,
	})

	intent_fsm:use({
		state = State.recover,
		enter = function()
			cancel_job()
			set_return_stage(ReturnStage.before_fluctuant)
			runtime.escape_route = nil
			runtime.current_intent = nil
			clear_forward_press_runtime()
			set_state(State.recover)
			set_phase("none")
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				return
			end

			if condition.health_ready() and condition.bullet_ready() then
				if should_enter_guard_home() then
					handle:set_next(State.guard_home)
				else
					handle:set_next(State.start_cruise)
				end
			end
		end,
	})

	assert(intent_fsm:init_ready(State), "intent fsm init_ready failed")
	return intent_fsm
end

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	option:set_handler(function(error)
		action:warn("while fetch option: " .. error)
	end)

	runtime.ours_zone = read_option("fsm_ours_zone", true)
	runtime.switch_interval = read_option("fsm_switch_interval", 5.0)

	configure_test_rule()
	setup_edges()

	if read_option("enable_goal_topic_forward", false) then
		action:switch_topic_forward(true)
	end
	action:bind(scheduler)

	local intent_fsm = create_intent_fsm()
	scheduler:append_task(function()
		while true do
			intent_fsm:spin_once()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			request:sleep(1.0)
			action:info(string.format(
				"fsm=%s phase=%s return_stage=%s stage=%s hp=%s bullet=%s rs=%s ls=%s",
				runtime.current_state,
				runtime.current_phase,
				tostring(runtime.return_stage),
				blackboard.game.stage,
				tostring(blackboard.user.health),
				tostring(blackboard.user.bullet),
				blackboard.play.rswitch,
				blackboard.play.lswitch
			))
		end
	end)

	action:info(ascii.banner)
	action:warn("FSM test endpoint loaded")
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	edges:spin()
	scheduler:spin_once()
end

on_exit = function()
	cancel_job()
	action:stop_navigation()
end

--- Callback for velocity topic from Nav2.
on_control = function(vx, vy, _)
	action:update_chassis_vel(vx, vy)
end
