---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")

local Map = require("map.core").new()
local bb = require("blackboard").singleton()

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

--- 内联测试图：起点原点，前哨点 (1.0, 0)
local kOrigin = Map:point("kOrigin", { x = 0.0, y = 0.0 })
local kAttackOutpost = Map:point("kAttackOutpost", { x = 1.0, y = 0.0 })

local function rough_navigate(_, to)
	action:navigate(to)
	action:info("navigate to " .. to.x .. ", " .. to.y)
	local timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(to, 0.5)
		end,
		timeout = 30,
	}
	return not timeout
end

Map:connect(kOrigin, kAttackOutpost) { rough_navigate, rough_navigate }

local kAttackMap = {
	map = Map,
	points = { kAttackOutpost = kAttackOutpost },
}

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("SB 测试模式，验证 attack-outpost 意图")

	clock:reset(blackboard.meta.timestamp)
	action:update_enable_control(true)
	action:gimbal_scan(-math.pi / 3, math.pi / 3)

	blackboard.rule.attack_map = kAttackMap
	blackboard.rule.attack_point = kAttackOutpost
	blackboard.context.current = kOrigin

	scheduler:append_task(task.robot_status)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("触发 attack-outpost 测试")
			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			request:sleep(6)
			action:relocalize()
			request:sleep(1)

			action:info("开始执行 attack-outpost 意图")
			scheduler:append_task(require("intent.attack-outpost").loop)
		end)

		while true do
			action:update_enable_control(blackboard.play.rswitch == "UP")

			switch_order:spin()
			request:yield()
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
