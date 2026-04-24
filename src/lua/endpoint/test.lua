---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")

local start_cruise = require("intent.start-cruise")
local keep_cruise = require("intent.keep-cruise")
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
	current_state = "idle",
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

	-- 坐标点位由地图/配置提供，这里不再覆写为 (0,0)。
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

local function start_navigation()
	local ok, message = action:restart_navigation({
		global_map = read_option("global_map", "rmuc"),
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

local function clear_navigate_history()
	local queue = blackboard.meta.navigate_point_queue
	if type(queue) ~= "table" then
		blackboard.meta.navigate_point_queue = {}
		return
	end

	for i = #queue, 1, -1 do
		queue[i] = nil
	end
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
	local function run_start_cruise_job()
		run_job("start_cruise", function()
			return start_cruise(runtime.ours_zone)
		end)
	end
	local function run_keep_cruise_job()
		run_job("keep_cruise", function()
			return keep_cruise(runtime.ours_zone, runtime.switch_interval)
		end)
	end
		local function run_escape_job()
			run_job("escape_to_home", function()
				return escape_to_home()
			end)
		end

	intent_fsm:use({
		state = State.idle,
		enter = function()
			cancel_job()
			set_state(State.idle)
			runtime.navigation_ready = false
		end,
		event = function(handle)
			if take_request("start") then
				local ok = start_navigation()
				runtime.navigation_ready = ok
			end

			if runtime.navigation_ready and blackboard.game.stage == "STARTED" then
				clear_navigate_history()
				handle:set_next(State.start_cruise)
			end
		end,
	})

	intent_fsm:use({
		state = State.start_cruise,
		enter = function()
			set_state(State.start_cruise)
			run_start_cruise_job()
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				cancel_job()
				handle:set_next(State.escape)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.keep_cruise)
				return
			end

			action:warn("fsm(start_cruise): 导航失败，重试当前状态")
			run_start_cruise_job()
		end,
	})

	intent_fsm:use({
		state = State.keep_cruise,
		enter = function()
			set_state(State.keep_cruise)
			run_keep_cruise_job()
		end,
		event = function(handle)
			if condition.low_health() or condition.low_bullet() then
				cancel_job()
				handle:set_next(State.escape)
				return
			end

			if not job.done then
				return
			end

			if job.success then
				return
			end

			action:warn("fsm(keep_cruise): 导航失败，重试当前状态")
			run_keep_cruise_job()
		end,
	})

	intent_fsm:use({
		state = State.escape,
		enter = function()
			set_state(State.escape)
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
			set_state(State.recover)
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
	runtime.switch_interval = read_option("fsm_switch_interval", 2.0)

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
				"fsm=%s stage=%s hp=%s bullet=%s rs=%s ls=%s",
				runtime.current_state,
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
