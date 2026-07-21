---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local order = require("util.order")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)
	action:gimbal_toward(0 / 0, 0 / 0)

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
	action:stop_navigation()

	--- 操作事件注册与响应
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			-- FIXME:
			-- scheduler:append_task(function()
			-- 	restart_navigation()
			-- 	request:sleep(5)
			--
			-- 	action:gimbal_nod(1)
			-- 	request:sleep(5)
			--
			-- 	blackboard.context.hint_intent = Intent.spikes
			-- 	clock:start_game_once()
			-- end)
		end)

		local last_stage = blackboard.game.stage
		blackboard.context.intent_to_return = Intent.spikes

		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")

			-- 比赛阶段检测
			-- @NOTE: 不要检测比赛结束喵，referee 有脏东西喵
			local stage = blackboard.game.stage
			if last_stage == GameStage.PREPARATION and stage ~= GameStage.PREPARATION then
				action:info("准备阶段结束喵")
				action:gimbal_nod(1)
				restart_navigation()
			end
			if last_stage ~= GameStage.STARTED and stage == GameStage.STARTED then
				action:info("比赛开始喵")
				clock:start_game_once()
				blackboard.context.hint_intent = Intent.spikes
			end
			last_stage = stage

			if stage == GameStage.STARTED then
				-- 血量检测
				local health = blackboard.user.health
				local health_limit = blackboard.rule.health_limit
				if health < health_limit then
					blackboard.context.hint_intent = Intent.supply
				end

				-- 弹药检测
				local bullet = blackboard.user.bullet
				local bullet_limit = blackboard.rule.bullet_limit
				if bullet < bullet_limit then
					blackboard.context.hint_intent = Intent.supply
				end
			end

			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(task.robot_status)

	--- 核心意图事件循环
	scheduler:append_task(function()
		local intent_fsm = fsm:new(blackboard.context.hint_intent)
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
			blackboard.context.last_intent = name
		end

		intent_fsm:use {
			state = Intent.nothing,
			enter = function()
				action:warn("⚠️你来到了没有意图的荒原")
				switch_intent(Intent.nothing)
			end,
			event = function(_) end,
		}
		intent_fsm:use {
			state = Intent.spikes,
			enter = function()
				action:info("进入 Spikes 模式，随机攻击路过的 Robot")
				switch_intent(Intent.spikes)
			end,
			event = function(_) end,
		}
		intent_fsm:use {
			state = Intent.supply,
			enter = function()
				action:warn("补给模式，速速回家")
				switch_intent(Intent.supply)
			end,
			event = function(_) end,
		}

		if not intent_fsm:init_ready(Intent) then
			error("意图状态机没有初始化完全，有未使用的意图")
		end

		while true do
			if blackboard.context.hint_intent ~= blackboard.context.last_intent then
				intent_fsm:start_on(blackboard.context.hint_intent)
				blackboard.context.last_intent = blackboard.context.hint_intent
			end

			intent_fsm:spin_once()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			action:info("当前比赛时间： " .. clock:game_time())
			request:sleep(5)
		end
	end)

	action:info(ascii.banner)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
