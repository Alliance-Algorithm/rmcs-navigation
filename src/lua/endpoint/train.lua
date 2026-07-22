local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")

local MapTrain = require("map.train")
local Map, Points = MapTrain.map, MapTrain.points

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local kTasks = {
	robot_status = require("task.robot_status"),
}

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:info("操作手训练......")

	clock:reset(blackboard.meta.timestamp)
	action:update_enable_control(false)

	local train_started = false

	local intent_handler = nil
	local intent = function()
		blackboard.context.current = Points.kA

		local kSpinDuration = 5
		while true do
			if blackboard.user.climb_failed then
				action:warn("climb failed, continuing from current position")
				blackboard.user.climb_failed = false
			end

			action:gimbal_toward(0, 0)
			local kPathA = Map:search(blackboard.context.current, Points.kB)
			for index, path_task in ipairs(kPathA) do
				action:info("execute kPathA task: " .. index)
				path_task()
			end

			action:gimbal_scan(0, 0)
			action:switch_motion_mode("attack")
			request:sleep(kSpinDuration)

			action:gimbal_toward(0, 0)
			local kPathB = Map:search(blackboard.context.current, Points.kC)
			for index, path_task in ipairs(kPathB) do
				action:info("execute kPathB task: " .. index)
				path_task()
			end

			action:gimbal_scan(0, 0)
			action:switch_motion_mode("attack")
			request:sleep(kSpinDuration)

			action:gimbal_toward(0, 0)
			local kPathC = Map:search(blackboard.context.current, Points.kE)
			for index, path_task in ipairs(kPathC) do
				action:info("execute kPathC task: " .. index)
				path_task()
			end

		action:gimbal_scan(0, 0)
		action:switch_motion_mode("attack")
		request:sleep(kSpinDuration)

		action:switch_motion_mode("normal")
		request:sleep(2.0)
		action:gimbal_toward(0, 0)
		local kPathD = Map:search(blackboard.context.current, Points.kF)
			for index, path_task in ipairs(kPathD) do
				action:info("execute kPathD task: " .. index)
				path_task()
			end

			action:gimbal_scan(0, 0)
			action:switch_motion_mode("attack")
			request:sleep(kSpinDuration)
		end
	end

	local toggle_train = function()
		if train_started then
			action:info("导航关闭")

			action:switch_motion_mode("normal")
			action:gimbal_toward(0, 0)

			action:stop_navigation()

			request:sleep(2)
			action:update_enable_control(false)
			action:switch_topic_forward(false)

			if intent_handler ~= nil then
				intent_handler.cancel()
				intent_handler = nil
			end

			train_started = false
		else
			action:info("导航启动")
			action:update_enable_control(true)

			action:gimbal_nod(2)

			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			request:sleep(2)
			action:switch_topic_forward(true)
			action:switch_motion_mode("attack")
			action:gimbal_scan(0, 0)

			intent_handler = scheduler:append_task(intent)

			train_started = true
		end
	end

	scheduler:append_task(kTasks.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, toggle_train)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
