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

local crossing_rough_terrain = require("task.crossing-rough-terrain")

local restart_navigation = function()
	action:info("导航即将重启")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}
end

local run_rough_terrain = function()
	scheduler:append_task(function()
		crossing_rough_terrain(true)
	end)
end

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:info(ascii.banner)
	action:warn("⚠️ TEST 模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:switch_topic_forward(true)

	action:bind(scheduler)

	restart_navigation()
	-- edges:on(blackboard.getter.rswitch, "UP", restart_navigation)
	edges:on(blackboard.getter.rswitch, "UP", run_rough_terrain)


	scheduler:append_task(function()
		while true do
			request:sleep(1)
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

--- 由 NAV2 发布的目标速度值，在此处理回调
on_control = function(x, y, _)
	action:update_chassis_vel(x, y)
end
