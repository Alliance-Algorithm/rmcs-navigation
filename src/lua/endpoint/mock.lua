---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local edges = require("util.edge").new()
local native = require("util.native")
local option = require("option")

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

	local global_map = option.global_map or "empty"
	native.run_command(
		string.format("ros2 launch rmcs-navigation static.launch.yaml global_map:=%s &", global_map)
	)
	action:info("static.launch.yaml launched with global_map:=" .. global_map)

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

--- 由 NAV2 发布的目标速度值，在此处理回调
on_control = function(x, y, _)
	action:update_chassis_vel(x, y)
end
