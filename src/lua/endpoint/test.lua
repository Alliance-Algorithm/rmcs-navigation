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
	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("⚠️ TEST 模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:update_enable_control(true)
	action:switch_topic_forward(true)

	restart_navigation()
	-- action:gimbal_scan(-0.2, 0.2)
	-- action:set_gimbal_yt(20)
	-- action:set_gimbal_pt(10)
	action:set_gimbal_yt(10)
	action:set_gimbal_pt(5)
	action:gimbal_scan(-0.4, 0)

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("Nod event appended")
			action:gimbal_nod(1)
		end)

		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")

			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			request:sleep(0.1)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
