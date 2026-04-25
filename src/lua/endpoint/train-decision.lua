---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")

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
	escape_route = nil,
	current_state = "idle",
	current_phase = "none",
	current_intent = nil,
	navigation_ready = false,
}

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

	rule.health_limit = read_option("fsm_health_limit", 210)
	rule.health_ready = read_option("fsm_health_ready", 400)
	rule.bullet_limit = read_option("fsm_bullet_limit", 40)
	rule.bullet_ready = read_option("fsm_bullet_ready", 300)

	-- Ours side sample points
	-- 暂时全为0 
	rule.resupply_zone.ours = { x = 0.0, y = 0.0 }   		--家
	rule.fluctuant_road_begin.ours = { x = 0.0, y = 0.0 }		--起伏路段起点
	rule.fluctuant_road_final.ours = { x = 0.0, y = 0.0 }		--起伏路段终点
	rule.one_step_begin.ours = { x = 0.0, y = 0.0 }			--一级台阶高点（先随便标个回家路上的点）
	rule.one_step_final.ours = { x = 0.0, y = 0.0 }			--一级台阶低点（先随便标个回家路上的点）
	rule.central_highland_near_fluctuant_road.ours = { x = 0.0, y = 0.0 }  	--高地靠近起伏路
	rule.central_highland_near_doghole.ours = { x = 0.0, y = 0.0 }			--高地靠近狗洞

	rule.central_highland_near_fluctuant_road.them = { x = 0.0, y = 0.0 }  	--高地靠近起伏路
	rule.central_highland_near_doghole.them = { x = 0.0, y = 0.0 }			--高地靠近狗洞
	rule.fluctuant_road_final.them = { x = 0.0, y = 0.0 }		--起伏路段终点
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
	blackboard.meta.fsm_phase = name
	action:info("fsm phase -> " .. name)
end

local function sync_intent_phase(intent)
	assert(intent ~= nil, "intent should exist before syncing phase")

	if type(intent.phase_name) == "function" then
		set_phase(intent:phase_name())
		return
	end

	set_phase("none")
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
		escape = "escape",
		recover = "recover",
	}

	local condition = blackboard.condition
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
		sync_intent_phase(runtime.current_intent)
	end
	local function create_keep_cruise_intent()
		runtime.current_intent = KeepCruiseIntent.new({
			ours_zone = runtime.ours_zone,
			switch_interval = runtime.switch_interval,
		})
		sync_intent_phase(runtime.current_intent)
	end
	local function run_current_intent_job()
		assert(runtime.current_intent ~= nil, "current intent should exist before running intent job")
		sync_intent_phase(runtime.current_intent)
		runtime.current_intent:run(run_job)
	end
	local function enter_escape(handle)
		local route = "direct"
		if runtime.current_intent ~= nil then
			route = runtime.current_intent:escape_route()
		end
		runtime.escape_route = route
		cancel_job()
		handle:set_next(State.escape)
	end

	intent_fsm:use({
		state = State.idle,
		enter = function()
			cancel_job()
			runtime.escape_route = nil
			runtime.current_intent = nil
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
				handle:set_next(State.start_cruise)
			end
		end,
	})

	intent_fsm:use({
		state = State.start_cruise,
		enter = function()
			set_state(State.start_cruise)
			create_start_cruise_intent()
			run_current_intent_job()
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				if runtime.current_intent:advance() then
					run_current_intent_job()
				else
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
			create_keep_cruise_intent()
			run_current_intent_job()
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				enter_escape(handle)
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
		state = State.escape,
		enter = function()
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
			runtime.escape_route = nil
			runtime.current_intent = nil
			set_state(State.recover)
			set_phase("none")
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				return
			end

			if condition.health_ready() and condition.bullet_ready() then
				handle:set_next(State.start_cruise)
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
				"fsm=%s phase=%s stage=%s hp=%s bullet=%s rs=%s ls=%s",
				runtime.current_state,
				runtime.current_phase,
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
