---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local edges = require("util.edge").new()

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

	action:restart_navigation {
		global_map = "战队",
		launch_livox = false,
		launch_odin1 = false,
		use_sim_time = false,
	}

	scheduler:append_task(function()
		request:sleep(1)
		action:relocalize()
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
