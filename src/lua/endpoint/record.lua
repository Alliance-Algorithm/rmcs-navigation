---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")

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
	action:warn("录制模式，rswitch 中→上→中 切换录制")
	action:update_enable_control(false)

	action:info("开启导航中......")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}

	local recording = false

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			local ok = action:toggle_record()
			if not ok then
				action:warn("录制切换失败")
				return
			end

			action:update_enable_control(true)
			if recording then
				action:info("关闭录制")
				action:gimbal_nod(2)
				recording = false
			else
				action:info("开始录制")
				action:gimbal_nod(1)
				recording = true
			end

			request:sleep(2)
			action:update_enable_control(false)
		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function()
	action:abort_record()
end
