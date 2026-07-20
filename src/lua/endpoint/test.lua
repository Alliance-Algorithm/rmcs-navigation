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
	action:update_enable_control(true)
	action:switch_topic_forward(true)

	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("Nod event appended")
			action:gimbal_nod(1)
			action:request_chassis_climb("START_ONE_STEP")
			action:info(string.format("start climb"))
			request:sleep(10)
			action:request_chassis_climb("IDLE")
		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			action:info(string.format("climb active=%s result=%s state=%s",
            blackboard.user.climb_active,
           	blackboard.user.climb_result,
            blackboard.user.climb_state))
			request:sleep(1)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
