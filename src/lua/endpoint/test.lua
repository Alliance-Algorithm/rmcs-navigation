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

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("⚠️ 重定位测试模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:switch_navigation(true)
	action:switch_topic_forward(true)

	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("导航即将重启")
			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
				launch_relocation = true,
			}
			scheduler:append_task(function()
        		request:sleep(5.0)
        		action:relocalize_initial(0.0, 0.0, 0.0)
 			 end)
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
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	scheduler:spin_once()
end

on_exit = function() end
