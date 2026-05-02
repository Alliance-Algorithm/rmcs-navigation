---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")

local CrossRoadIntent = require("intent.train.cross-road")
local EscapeToHomeIntent = require("intent.train.escape-to-home")
local KeepCruiseIntent = require("intent.train.keep-cruise")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local edges = require("util.edge").new()
local BlackboardLogger = require("util.blackboard_logger")

---
--- Export Context
---

blackboard = require("blackboard").singleton()

local runtime = {
	ours_zone = true,
	switch_interval = 5.0,
	current_state = "idle",
	current_phase = "none",
	current_intent_kind = nil,
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

local function configure_train_rule()
	local rule = blackboard.rule

	rule.health_limit = read_option("health_limit", rule.health_limit)
	rule.health_ready = read_option("health_ready", rule.health_ready)
	rule.bullet_limit = read_option("bullet_limit", rule.bullet_limit)
	rule.bullet_ready = read_option("bullet_ready", rule.bullet_ready)

	rule.resupply_zone.ours = { x = 0.0, y = 0.0 }
	rule.road_zone_begin.ours = { x = 2.6, y = 2.1 }
	rule.road_zone_final.ours = { x = 4.7, y = 3.6 }
	rule.central_highland_near_fluctuant_road.ours = { x = 5.7, y = 1.9 }
	rule.central_highland_near_doghole.ours = { x = 4.8, y = -1.4 }
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
			action:fuck(string.format("train fsm job '%s' failed:\n%s", name, result))
			return
		end

		job.success = (result ~= false)
		if not job.success then
			action:warn(string.format("train fsm job '%s' finished with false", name))
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
	action:info("train fsm state -> " .. name)
end

local function set_phase(name)
	if runtime.current_phase == name then
		return
	end
	runtime.current_phase = name
	action:info("train fsm phase -> " .. name)
end

local function start_navigation()
	local global_map = read_option("global_map", "train_map")
	local ok, message = action:restart_navigation({
		global_map = global_map,
		launch_livox = read_option("launch_livox", true),
		launch_odin1 = read_option("launch_odin1", false),
		use_sim_time = read_option("use_sim_time", false),
	})
	if not ok then
		action:fuck("train restart_navigation 触发失败: " .. tostring(message))
	end

	return ok, message
end

local function setup_edges()
	edges:on(blackboard.getter.rswitch, "UP", function()
		-- 手动测试入口：仅允许右拨杆向上触发启动。
		requests.start = true
	end)
end

local function create_intent(kind)
	if kind == "cross_road" then
		return CrossRoadIntent.new({
			ours_zone = runtime.ours_zone,
			forward_center = true,
		})
	end

	if kind == "keep_cruise" then
		return KeepCruiseIntent.new({
			ours_zone = runtime.ours_zone,
			switch_interval = runtime.switch_interval,
		})
	end

	if kind == "escape" then
		return EscapeToHomeIntent.new({
			ours_zone = runtime.ours_zone,
		})
	end

	error("unknown train intent kind: " .. tostring(kind))
end

local function replace_intent(kind, force)
	assert(type(kind) == "string", "intent kind should be a string")
	if not force and runtime.current_intent_kind == kind and runtime.current_intent ~= nil then
		return
	end

	clear_current_intent()
	runtime.current_intent_kind = kind
	runtime.current_intent = create_intent(kind)
	action:info("train intent -> " .. kind)
end

local intent_ctx = {
	run_job = run_job,
	cancel_job = cancel_job,
	job_state = function()
		return job
	end,
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

local function create_endpoint_fsm()
	local State = {
		idle = "idle",
		cross_road = "cross_road",
		keep_cruise = "keep_cruise",
		escape = "escape",
		recover = "recover",
	}

	local condition = blackboard.condition
	local endpoint_fsm = fsm:new(State.idle)

	endpoint_fsm:use({
		state = State.idle,
		enter = function()
			clear_current_intent()
			runtime.navigation_ready = false
			set_state(State.idle)
			set_phase("none")
		end,
		event = function(handle)
			if take_request("start") then
				local ok = start_navigation()
				runtime.navigation_ready = ok
			end

			if runtime.navigation_ready then
				handle:set_next(State.cross_road)
			end
		end,
	})

	endpoint_fsm:use({
		state = State.cross_road,
		enter = function()
			set_state(State.cross_road)
			replace_intent("cross_road", true)
		end,
		event = function(handle)
			if condition.low_health() then
				clear_current_intent()
				handle:set_next(State.escape)
				return
			end

			local status = spin_current_intent()
			if status == "failed" then
				action:warn("train fsm(cross_road): 通过公路区失败，重试")
				replace_intent("cross_road", true)
				return
			end

			if status == "success" then
				clear_current_intent()
				handle:set_next(State.keep_cruise)
			end
		end,
	})

	endpoint_fsm:use({
		state = State.keep_cruise,
		enter = function()
			set_state(State.keep_cruise)
			replace_intent("keep_cruise", true)
		end,
		event = function(handle)
			if condition.low_health() then
				clear_current_intent()
				handle:set_next(State.escape)
				return
			end

			local status = spin_current_intent()
			if status == "failed" then
				action:warn("train fsm(keep_cruise): 巡航失败，重试")
				replace_intent("keep_cruise", true)
			end
		end,
	})

	endpoint_fsm:use({
		state = State.escape,
		enter = function()
			set_state(State.escape)
			replace_intent("escape", true)
		end,
		event = function(handle)
			local status = spin_current_intent()
			if status == "failed" then
				action:warn("train fsm(escape): 回补给点失败，重试")
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
			set_state(State.recover)
			set_phase("none")
		end,
		event = function(handle)
			if condition.low_health() then
				return
			end

			if condition.health_ready() and condition.bullet_ready() then
				handle:set_next(State.cross_road)
			end
		end,
	})

	assert(endpoint_fsm:init_ready(State), "train endpoint fsm init_ready failed")
	return endpoint_fsm
end

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	option:set_handler(function(error)
		action:warn("while fetch option: " .. error)
	end)

	runtime.ours_zone = true
	runtime.switch_interval = read_option("fsm_switch_interval", 5.0)

	configure_train_rule()
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

	-- BlackboardLogger.attach(scheduler, blackboard)

	action:info(ascii.banner)
	action:warn("train FSM endpoint loaded")
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
