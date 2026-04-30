---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local Map = require("map")
local option = require("option")

local EscapeToHomeIntent = require("intent.competion.escape-to-home")
local ForwardPressIntent = require("intent.competion.forward-press")
local GuardHomeIntent = require("intent.competion.guard-home")
local KeepCruiseIntent = require("intent.competion.keep-cruise")
local Region = require("region")
local StartCruiseIntent = require("intent.competion.start-cruise")

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
	region = nil,
	region_name = "unknown",
	region_phase = Region.Phase.unknown,
	current_state = "idle",
	current_phase = "none",
	current_intent_kind = nil,
	current_intent = nil,
	navigation_ready = false,
	escape_route = nil,
	forward_press_mode = nil,
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
	local value = rawget(option, name)
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

	rule.resupply_zone.ours = { x = 0.0, y = 0.0 }
	rule.fluctuant_road_begin.ours = { x = 0.0, y = 0.0 }
	rule.fluctuant_road_final.ours = { x = 0.0, y = 0.0 }
	rule.one_step_begin.ours = { x = 0.0, y = 0.0 }
	rule.one_step_final.ours = { x = 0.0, y = 0.0 }
	rule.central_highland_near_fluctuant_road.ours = { x = 0.0, y = 0.0 }
	rule.central_highland_near_doghole.ours = { x = 0.0, y = 0.0 }
	rule.base_left_gain_point.ours = { x = 0.0, y = 0.0 }
	rule.base_right_gain_point.ours = { x = 0.0, y = 0.0 }
	rule.fortress.ours = { x = 0.0, y = 0.0 }

	rule.central_highland_near_fluctuant_road.them = { x = 0.0, y = 0.0 }
	rule.central_highland_near_doghole.them = { x = 0.0, y = 0.0 }
	rule.fluctuant_road_final.them = { x = 0.0, y = 0.0 }
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

local function clear_current_intent()
	cancel_job()
	runtime.current_intent_kind = nil
	runtime.current_intent = nil
	runtime.current_phase = "none"
end

local function take_request(name)
	local value = requests[name]
	requests[name] = false
	return value
end

local function set_state(name)
	if runtime.current_state == name then
		return
	end
	runtime.current_state = name
	blackboard.meta.fsm_state = name
	action:info("fsm state -> " .. name)
end

local function set_phase(name)
	if runtime.current_phase == name then
		return
	end
	runtime.current_phase = name
	action:info("fsm phase -> " .. name)
end

local function sync_region()
	local region, region_name = Region.current()
	runtime.region = region
	runtime.region_name = region_name
	runtime.region_phase = Region.phase(region)
	blackboard.meta.region = region_name
	blackboard.meta.fsm_return_stage = Region.return_stage(region)
end

local function configure_region_map(name)
	assert(type(name) == "string", "global_map should be a string")
	local ok, loaded_or_error = pcall(Map.singleton, name)
	if not ok then
		action:fuck("load region map failed: " .. tostring(loaded_or_error))
		return false, loaded_or_error
	end
	action:info("region map -> " .. Map.current_name())
	return true, loaded_or_error
end

local function start_navigation()
	local global_map = read_option("global_map", "rmuc")
	local ok, load_error = configure_region_map(global_map)
	if not ok then
		return false, load_error
	end

	local ok, message = action:restart_navigation({
		global_map = global_map,
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

local function should_enter_guard_home()
	local condition = blackboard.condition
	return condition.game_close_to_end() or condition.base_in_danger()
end

local function guard_home_target()
	if blackboard.condition.fortress_occupied() then
		return "cruise_in_front_of_base"
	end
	return "occupy_fortress"
end

local last_double_damage_activated = false
local last_big_energy_mechanism_activated = false
local last_small_energy_mechanism_activated = false

local function select_forward_press_mode()
	local condition = blackboard.condition
	local dart_hit_first_time = condition.dart_hit_first_time()
	local double_damage_activated = condition.double_damage_activated()
	local big_energy_mechanism_activated = condition.big_energy_mechanism_activated()
	local small_energy_mechanism_activated = condition.small_energy_mechanism_activated()
	local double_damage_rising = double_damage_activated and not last_double_damage_activated
	local big_energy_rising = big_energy_mechanism_activated and not last_big_energy_mechanism_activated
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

local function create_intent(kind)
	if kind == "start_cruise" then
		return StartCruiseIntent.new({
			ours_zone = runtime.ours_zone,
		})
	end

	if kind == "keep_cruise" then
		return KeepCruiseIntent.new({
			ours_zone = runtime.ours_zone,
			switch_interval = runtime.switch_interval,
		})
	end

	if kind == "guard_home" then
		return GuardHomeIntent.new({
			ours_zone = runtime.ours_zone,
		})
	end

	if kind == "forward_press" then
		assert(type(runtime.forward_press_mode) == "string", "forward_press_mode should be set")
		return ForwardPressIntent.new({
			mode = runtime.forward_press_mode,
			switch_interval = runtime.switch_interval,
			duration = forward_press_duration,
		})
	end

	if kind == "escape" then
		assert(type(runtime.escape_route) == "string", "escape_route should be set")
		return EscapeToHomeIntent.new({
			route = runtime.escape_route,
		})
	end

	error("unknown intent kind: " .. tostring(kind))
end

local function replace_intent(kind, force)
	assert(type(kind) == "string", "intent kind should be a string")
	if not force and runtime.current_intent_kind == kind and runtime.current_intent ~= nil then
		return
	end

	clear_current_intent()
	runtime.current_intent_kind = kind
	runtime.current_intent = create_intent(kind)
	action:info("intent -> " .. kind)
end

local intent_ctx = {
	run_job = run_job,
	cancel_job = cancel_job,
	job_state = function()
		return job
	end,
	region = function()
		return runtime.region
	end,
	region_phase = function()
		return runtime.region_phase
	end,
	guard_home_target = guard_home_target,
}

local function sync_intent_phase()
	if runtime.current_intent == nil or type(runtime.current_intent.phase_name) ~= "function" then
		set_phase("none")
		return
	end
	set_phase(runtime.current_intent:phase_name())
end

local function spin_current_intent()
	if runtime.current_intent == nil then
		set_phase("none")
		return nil
	end

	local status = runtime.current_intent:spin(intent_ctx)
	sync_intent_phase()
	return status
end

local function choose_active_intent_kind()
	if should_enter_guard_home() then
		return "guard_home"
	end

	if runtime.current_intent_kind == "forward_press" and runtime.current_intent ~= nil then
		return "forward_press"
	end

	if Region.is_before_fluctuant(runtime.region) or Region.is_on_fluctuant(runtime.region) then
		return "start_cruise"
	end

	local mode = select_forward_press_mode()
	if mode ~= nil then
		runtime.forward_press_mode = mode
		return "forward_press"
	end

	return "keep_cruise"
end

local function create_endpoint_fsm()
	local State = {
		idle = "idle",
		active = "active",
		escape = "escape",
		recover = "recover",
	}

	local condition = blackboard.condition
	local endpoint_fsm = fsm:new(State.idle)

	endpoint_fsm:use({
		state = State.idle,
		enter = function()
			clear_current_intent()
			runtime.escape_route = nil
			runtime.forward_press_mode = nil
			runtime.navigation_ready = false
			set_state(State.idle)
			set_phase("none")
		end,
		event = function(handle)
			sync_region()

			if take_request("start") then
				local ok = start_navigation()
				runtime.navigation_ready = ok
			end

			if runtime.navigation_ready and blackboard.game.stage == "STARTED" then
				handle:set_next(State.active)
			end
		end,
	})

	endpoint_fsm:use({
		state = State.active,
		enter = function()
			set_state(State.active)
		end,
		event = function(handle)
			sync_region()

			if condition.low_health() or condition.low_bullet() then
				runtime.escape_route = Region.escape_route(runtime.region)
				clear_current_intent()
				handle:set_next(State.escape)
				return
			end

			local desired_kind = choose_active_intent_kind()
			if runtime.current_intent_kind ~= desired_kind or runtime.current_intent == nil then
				replace_intent(desired_kind, true)
			end

			local status = spin_current_intent()
			if status == "failed" then
				action:warn(string.format(
					"fsm(active:%s): 当前 intent 失败，重建 %s",
					runtime.current_phase,
					desired_kind
				))
				replace_intent(desired_kind, true)
				return
			end

			if status == "success" then
				if runtime.current_intent_kind == "forward_press" then
					runtime.forward_press_mode = nil
				end
				clear_current_intent()
			end
		end,
	})

	endpoint_fsm:use({
		state = State.escape,
		enter = function()
			set_state(State.escape)
			runtime.escape_route = runtime.escape_route or Region.escape_route(runtime.region)
			replace_intent("escape", true)
		end,
		event = function(handle)
			sync_region()

			local status = spin_current_intent()
			if status == "failed" then
				action:warn("fsm(escape): 回家失败，重试 escape intent")
				replace_intent("escape", true)
				return
			end

			if status == "success" then
				clear_current_intent()
				handle:set_next(State.recover)
			end
		end,
	})

	endpoint_fsm:use({
		state = State.recover,
		enter = function()
			clear_current_intent()
			runtime.escape_route = nil
			runtime.forward_press_mode = nil
			set_state(State.recover)
			set_phase("none")
		end,
		event = function(handle)
			sync_region()

			if condition.low_health() or condition.low_bullet() then
				return
			end

			if condition.health_ready() and condition.bullet_ready() then
				handle:set_next(State.active)
			end
		end,
	})

	assert(endpoint_fsm:init_ready(State), "endpoint fsm init_ready failed")
	return endpoint_fsm
end

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	option:set_handler(function(error)
		action:warn("while fetch option: " .. error)
	end)

	runtime.ours_zone = read_option("fsm_ours_zone", true)
	runtime.switch_interval = read_option("fsm_switch_interval", 5.0)

	configure_test_rule()
	do
		local ok, err = configure_region_map(read_option("global_map", "rmuc"))
		assert(ok, "failed to configure region map: " .. tostring(err))
	end
	setup_edges()

	if read_option("enable_goal_topic_forward", false) then
		action:switch_topic_forward(true)
	end
	action:bind(scheduler)

	local endpoint_fsm = create_endpoint_fsm()
	scheduler:append_task(function()
		while true do
			endpoint_fsm:spin_once()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			request:sleep(1.0)
			action:info(string.format(
				"fsm=%s intent=%s phase=%s region=%s hp=%s bullet=%s rs=%s ls=%s",
				runtime.current_state,
				tostring(runtime.current_intent_kind),
				runtime.current_phase,
				runtime.region_name,
				tostring(blackboard.user.health),
				tostring(blackboard.user.bullet),
				blackboard.play.rswitch,
				blackboard.play.lswitch
			))
		end
	end)

	action:info(ascii.banner)
	action:warn("new FSM endpoint loaded")
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	edges:spin()
	scheduler:spin_once()
end

on_exit = function()
	clear_current_intent()
	action:stop_navigation()
end

on_control = function(vx, vy, _)
	action:update_chassis_vel(vx, vy)
end
