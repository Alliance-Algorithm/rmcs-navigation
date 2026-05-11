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

local edges = require("util.edge").new()

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

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	--- 高优先级事件中断检测
	scheduler:append_task(function()
		while true do
			action:switch_navigation(blackboard.play.rswitch == "UP")
			request:yield()
		end
	end)

	--- 核心意图事件循环
	scheduler:append_task(function()
		local Intent = {
			nothing = "nothing",
		}
		local intent_fsm = fsm:new(Intent.nothing)

		intent_fsm:use {
			state = Intent.nothing,
			enter = function()
				action:warn("⚠️你来到了没有意图的荒原")
			end,
			event = function(handle)
				handle:set_next(Intent.nothing)
			end,
		}
		if not intent_fsm:init_ready(Intent) then
			error("意图状态机没有初始化完全，有未使用的意图")
		end

		while true do
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
