---
--- Local Context
---

local api = require("api")
local ascii = require("util.ascii_art")
local clock = require("util.clock")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	api.info(ascii.banner)

	clock:reset(blackboard.meta.timestamp)

	scheduler:append_task(function()
		request:sleep(2.0)
		api.info("test endpoint: send first goal")
		api.send_target(1.0, 1.0)

		request:sleep(0.5)
		api.info("test endpoint: send duplicated goal")
		api.send_target(1.0, 1.0)

		request:sleep(0.5)
		api.info("test endpoint: replace with a new goal")
		api.send_target(2.0, 1.0)
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	scheduler:spin_once()
end

--- 由 NAV2 发布的目标速度值，在此处理回调
control_speed_callback = function(_, _, _) end
