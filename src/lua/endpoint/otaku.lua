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
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("御宅模式♥️，守护你的基地")

	action:update_enable_control(true)
	action:gimbal_scan(0, 0)
	action:switch_motion_mode("attack")

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("切换录制状态")
			action:toggle_record()
		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		local _ = edges.new()

		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")
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
