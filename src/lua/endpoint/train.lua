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
		blackboard.context.current = Points.kOrigin

		local kPoints = {
			Points.kAttackInLeft,
			Points.kAttackInRight,
			Points.kUnderRune,
			Points.kAttackAtHole,
		}
		while true do
			for _, point in ipairs(kPoints) do
				action:gimbal_suspend()

				for index, path_task in ipairs(Map:search(blackboard.context.current, point)) do
					action:info("Execute path task: " .. index)
					if not path_task() then
						action:fuck("Failed to follow path, return")
						return
					end
				end
				blackboard.context.current = point

				action:gimbal_scan(0, 0)
				request:sleep(3)
			end
		end
	end

	local toggle_train = function()
		if train_started then
			action:info("导航关闭")

			action:switch_motion_mode("normal")
			action:gimbal_suspend()

			request:sleep(0.5)
			action:update_enable_control(false)
			action:switch_topic_forward(false)

			action:stop_navigation()

			if intent_handler ~= nil then
				intent_handler.cancel()
				intent_handler = nil
			end

			train_started = false
		else
			action:info("导航启动")
			action:update_enable_control(true)
			action:gimbal_suspend()

			action:restart_navigation {
				global_map = "战队",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			request:sleep(2)
			action:relocalize()
			action:switch_topic_forward(true)
			action:switch_motion_mode("attack")

			request:sleep(3)

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

on_exit = function()
	action:stop_navigation()
end
