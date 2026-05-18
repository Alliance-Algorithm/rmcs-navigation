---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")
local edges = require("util.edge")

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
	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("⚠️ TEST 模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:switch_navigation(true)
	action:switch_topic_forward(true)

	action:info("导航即将重启")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			-- action:gimbal_scan(-1, 1)
			action:switch_motion_mode("attack")
		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		local _ = edges.new()

		while true do
			action:switch_navigation(blackboard.play.rswitch == "UP")
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			-- local hp = blackboard.user.health
			-- action:info("hp: " .. hp)

			request:sleep(1)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	scheduler:spin_once()
end

on_exit = function() end
