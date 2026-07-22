---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")
local edges = require("util.edge")
local option = require("option")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

local restart_navigation = function()
	action:info("导航即将重启")
	action:restart_navigation {
		global_map = "rmuc",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}
end

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)

	action:info(ascii.banner)
	action:warn("⚠️ TEST 模式，别上场哦")

	action:update_enable_control(true)

	option:set_handler(function(error)
		action:fuck("while fetch option: " .. error)
	end)

	if option.enable_goal_topic_forward then
		action:switch_topic_forward(true)
	end

	local restart_navigation = function()
		action:info("导航即将重启")
		action:restart_navigation {
			global_map = "rmuc",
			launch_livox = true,
			launch_odin1 = false,
			use_sim_time = false,
		}
	end
	restart_navigation()

	-- action:gimbal_scan(-0.2, 0.2)
	-- action:set_gimbal_yt(20)
	-- action:set_gimbal_pt(10)


	scheduler:append_task(task.robot_status)
	-- scheduler:append_task(function()
	-- local switch_order = order.new(blackboard.getter.rswitch, 0.5)
	-- local posture_index = 0
	-- local events = {
	-- SentryEvent.SWITCH_POSE_ATTACK,
	-- SentryEvent.SWITCH_POSE_DEFENSE,
	-- SentryEvent.SWITCH_POSE_MOVE,
	-- SentryEvent.SWITCH_POSE_POWERED_ATTACK,
	-- SentryEvent.SWITCH_POSE_POWERED_DEFENSE,
	-- SentryEvent.SWITCH_POSE_POWERED_MOVE,
	-- }
	-- local event_names = { "ATTACK", "DEFENSE", "MOVE", "POWERED_ATTACK", "POWERED_DEFENSE", "POWERED_MOVE" }
	--
	-- switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
	-- action:info("中上中触发: 重启导航")
	-- restart_navigation()
	-- request:sleep(3)
	-- action:gimbal_nod(1)
	-- end)
	--
	-- switch_order:on({ "MIDDLE", "DOWN", "MIDDLE" }, function()
	-- posture_index = (posture_index % #events) + 1
	-- local ev = events[posture_index]
	-- action:info("中下中触发: push " .. event_names[posture_index])
	-- action:push_sentry_event(ev)
	-- end)
	--
	-- while true do
	-- action:switch_navigation(blackboard.play.rswitch == "UP")
	-- switch_order:spin()
	-- request:yield()
	-- end
	-- end)


	action:set_gimbal_yt(2)
	action:set_gimbal_pt(2)
	action:gimbal_scan(0, 0)
	scheduler:append_task(function()
		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")
			request:sleep(0.1)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
