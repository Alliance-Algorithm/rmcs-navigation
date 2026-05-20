---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local order = require("util.order")
local edges = require("util.edge")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

local Intent = {
	nothing = "nothing",
	otaku = "otaku",
	spikes = "spikes",
}
local hint_intent = Intent.nothing

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)
	action:info("use decision: '" .. option.decision .. "'")

	option:set_handler(function(error)
		action:fuck("while fetch option: " .. error)
	end)

	if option.enable_goal_topic_forward then
		action:switch_topic_forward(true)
	end

	--- 操作事件注册与响应
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("导航即将重启")
			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}
		end)

		local last_stage = blackboard.game.stage

		while true do
			local stage = blackboard.game.stage
			if last_stage == GameStage.NOT_START and stage ~= GameStage.NOT_START then
				action:info("导航即将重启")
				action:restart_navigation {
					global_map = "empty",
					launch_livox = true,
					launch_odin1 = false,
					use_sim_time = false,
				}
			end
			if last_stage ~= GameStage.STARTED and stage == GameStage.STARTED then
				hint_intent = Intent.otaku
			end
			last_stage = stage

			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(task.robot_status)

	--- 核心意图事件循环
	scheduler:append_task(function()
		local intent_fsm = fsm:new(hint_intent)
		local last_intent = nil
		local last_handle = nil

		local switch_intent = function(name)
			if last_handle ~= nil then
				last_handle.cancel()
			end

			if name ~= "" and name ~= nil then
				local intent = require("intent." .. name)
				last_handle = scheduler:append_task(intent.loop)
			else
				last_handle = nil
			end
			last_intent = name
		end

		intent_fsm:use {
			state = Intent.otaku,
			enter = function()
				action:info("进入 otaku 模式，请坐好放宽，哨兵会守护你的基地")
				action:switch_motion_mode("attack")
				action:gimbal_scan(0, 0)
			end,
			event = function() end,
		}

		intent_fsm:use {
			state = Intent.nothing,
			enter = function()
				action:warn("⚠️你来到了没有意图的荒原")
			end,
			event = function(_) end,
		}
		intent_fsm:use {
			state = Intent.spikes,
			enter = function()
				action:info("进入 Spikes 模式，随机攻击路过的 Robot")
				switch_intent(Intent.spikes)
			end,
			event = function() end,
		}
		if not intent_fsm:init_ready(Intent) then
			error("意图状态机没有初始化完全，有未使用的意图")
		end

		while true do
			if hint_intent ~= last_intent then
				intent_fsm:start_on(hint_intent)
				last_intent = hint_intent
			end

			intent_fsm:spin_once()
			request:yield()
		end
	end)

	action:info(ascii.banner)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
