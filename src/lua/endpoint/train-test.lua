---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local EscapeRoute = require("train.escape_route")
local TrainMap = require("train_map")
local TrainRule = require("train.rule")

local CrossRoadIntent = require("intent.train.cross-road")
local EscapeToHomeIntent = require("intent.train.escape-to-home")
local KeepCruiseIntent = require("intent.train.keep-cruise")

local Scheduler = require("util.scheduler")
local Job = require("train.job")
local scheduler = Scheduler.new()
local request = Scheduler.request

local edges = require("util.edge").new()

---
--- Export Context
---

blackboard = require("blackboard").singleton()

local runtime = {
	ours_zone = true,
	switch_interval = 5.0,
	region = nil,
	region_name = "unknown",
	escape_route = nil,
	current_state = "idle",
	current_phase = "none",
	current_intent_kind = nil,
	current_intent = nil,
	navigation_ready = false,
}

local requests = {
	start = false,
}

local job = Job.new({
	scheduler = scheduler,
	action = action,
	label = "train fsm",
})
local last_control_log_time = nil
local restart_switch_pattern = {
	step = 1,
	start_time = nil,
	last = nil,
}

local function read_option(name, fallback)
	local value = rawget(option, name)
	if value == nil then
		return fallback
	end
	return value
end

local function clear_current_intent()
	job:cancel()
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

local function current_train_region()
	local map = TrainMap.singleton(read_option("global_map", "train_map"))
	local region = map:locate({
		x = blackboard.user.x,
		y = blackboard.user.y,
	})
	return region, map.names[region] or "unknown"
end

local function sync_train_region()
	local region, region_name = current_train_region()
	runtime.region = region
	runtime.region_name = region_name
	blackboard.meta.region = region_name
end

local function select_escape_route()
	if runtime.region == nil then
		sync_train_region()
	end

	return EscapeRoute.select({
		region = runtime.region,
		position = {
			x = blackboard.user.x,
			y = blackboard.user.y,
		},
		rule = blackboard.rule,
		ours_zone = runtime.ours_zone,
	})
end

local function start_navigation()
	local global_map = read_option("global_map", "train_map")
	local ok, load_error = pcall(TrainMap.singleton, global_map)
	if not ok then
		action:fuck("train load region map failed: " .. tostring(load_error))
		return false, load_error
	end

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
			route = runtime.escape_route or select_escape_route(),
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
	run_job = function(name, fn)
		job:run(name, fn)
	end,
	cancel_job = function()
		job:cancel()
	end,
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

local function escape_if_low_health(handle, State)
	if not blackboard.condition.low_health() then
		return false
	end

	runtime.escape_route = select_escape_route()
	clear_current_intent()
	handle:set_next(State.escape)
	return true
end

local function retry_failed_intent(kind, message)
	action:warn(message)
	replace_intent(kind, true)
end

local function create_endpoint_fsm()
	local State = {
		idle = "idle",
		cross_road = "cross_road",
		keep_cruise = "keep_cruise",
		escape = "escape",
		recover = "recover",
	}

	local endpoint_fsm = fsm:new(State.idle)

	local function use_intent_state(args)
		endpoint_fsm:use({
			state = args.state,
			enter = function()
				set_state(args.state)
				if args.prepare then
					args.prepare()
				end
				replace_intent(args.intent, true)
			end,
			event = function(handle)
				sync_train_region()

				if args.escape_on_low_health and escape_if_low_health(handle, State) then
					return
				end

				local status = spin_current_intent()
				if status == "failed" then
					retry_failed_intent(args.intent, args.failure_message)
					return
				end

				if status == "success" and args.next_state then
					clear_current_intent()
					handle:set_next(args.next_state)
				end
			end,
		})
	end

	endpoint_fsm:use({
		state = State.idle,
		enter = function()
			clear_current_intent()
			runtime.escape_route = nil
			runtime.navigation_ready = false
			set_state(State.idle)
			set_phase("none")
		end,
		event = function(handle)
			sync_train_region()

			if take_request("start") then
				local ok = start_navigation()
				runtime.navigation_ready = ok
			end

			if runtime.navigation_ready then
				--handle:set_next(State.cross_road)
			end
		end,
	})

	use_intent_state({
		state = State.cross_road,
		intent = "cross_road",
		escape_on_low_health = true,
		failure_message = "train fsm(cross_road): 通过公路区失败，重试",
		next_state = State.keep_cruise,
	})

	use_intent_state({
		state = State.keep_cruise,
		intent = "keep_cruise",
		escape_on_low_health = true,
		failure_message = "train fsm(keep_cruise): 巡航失败，重试",
	})

	use_intent_state({
		state = State.escape,
		intent = "escape",
		prepare = function()
			runtime.escape_route = runtime.escape_route or select_escape_route()
		end,
		failure_message = "train fsm(escape): 回补给点失败，重试",
		next_state = State.recover,
	})

	endpoint_fsm:use({
		state = State.recover,
		enter = function()
			clear_current_intent()
			runtime.escape_route = nil
			set_state(State.recover)
			set_phase("none")
		end,
		event = function(handle)
			sync_train_region()

			if blackboard.condition.low_health() then
				return
			end

			if blackboard.condition.health_ready() then
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

	edges:on(blackboard.getter.rswitch, "UP", restart_navigation)

	runtime.ours_zone = true
	runtime.switch_interval = read_option("fsm_switch_interval", 5.0)

	TrainRule.configure(blackboard.rule, read_option)
	requests.start = true

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
			sync_train_region()
			action:info(string.format(
				"train position x=%.2f y=%.2f region=%s escape_route=%s",
				blackboard.user.x,
				blackboard.user.y,
				runtime.region_name,
				tostring(runtime.escape_route or select_escape_route())
			))
		end
	end)

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
	local now = clock:now()
	if last_control_log_time == nil or now - last_control_log_time >= 2.0 then
		action:info(string.format("NAV2 目标速度: vx=%.3f, vy=%.3f", vx, vy))
		last_control_log_time = now
	end

	action:update_chassis_vel(vx, vy)
end
