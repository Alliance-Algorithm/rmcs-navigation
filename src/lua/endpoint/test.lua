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

local restart_navigation = function()
	action:info("导航即将重启")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}
	-- action:exchange_17mm_bullet(100)
	action:confirm_revive();
	-- action:switch_mode(1);
end

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:warn("⚠️ TEST 模式，别上场哦")

	clock:reset(blackboard.meta.timestamp)
	action:switch_topic_forward(true)

	restart_navigation()
	edges:on(blackboard.getter.rswitch, "UP", restart_navigation)

	scheduler:append_task(function()
		while true do
			request:sleep(1)
			-- action:info("limit: " .. blackboard.user.chassis_power_limit)
			action:info(string.format("bullet %d health %d mode %d can_confirm_free_revive %s gimbal_commmand_keyboad %d x %d y %d",
			blackboard.user.bullet, blackboard.user.health, blackboard.game.sentry_mode, blackboard.game.can_confirm_free_revive, blackboard.map_command.keyboard, blackboard.map_command.x, blackboard.map_command.y))
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
	local now = clock:now()
	if last_control_log_time == nil or now - last_control_log_time >= 2.0 then
		action:info(string.format("NAV2 目标速度: vx=%.3f, vy=%.3f", x, y))
		last_control_log_time = now
	end

	action:update_chassis_vel(x, y)
end
