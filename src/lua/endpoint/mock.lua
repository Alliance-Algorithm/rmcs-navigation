---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local edges = require("util.edge").new()
local native = require("util.native")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:info(ascii.banner)
	action:warn("⚠️ MOCK 模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:switch_topic_forward(true)

	native.run("ros2 launch rmcs-navigation static.launch.yaml &")
	action:info("static.launch.yaml launched")

	scheduler:append_task(function()
		while true do
			request:sleep(0.5)
			-- action:info("limit: " .. blackboard.user.chassis_power_limit)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	edges:spin()
	scheduler:spin_once()
end

on_exit = function()
	action:stop_navigation()
end
