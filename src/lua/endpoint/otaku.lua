---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")

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
	action:warn("什么也不动的固定炮台模式")

	action:update_enable_control(true)

	action:set_gimbal_pt(2)
	action:set_gimbal_yt(2)
	action:gimbal_scan(0, 0)
	action:switch_motion_mode("attack")

	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")
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
