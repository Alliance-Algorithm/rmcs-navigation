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

--- @TODO: 自定义巡逻点
local patrol_points = {
	{ x = 0, y = 0 },
	{ x = 0, y = 1.0 },
}

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)
	action:gimbal_toward(0 / 0, 0 / 0)
	action:switch_topic_forward(true)
	action:info(ascii.banner)
	action:warn("⚠️ TRAIN 模式")

	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}

	scheduler:append_task(function()
		action:gimbal_nod(1)
	end)

	scheduler:append_task(task.robot_status)

	scheduler:append_task(function()
		while true do
			local enable = blackboard.play.rswitch == "UP"
				and not blackboard.user.enemy_visible
				and blackboard.user.health > 0
			action:switch_navigation(enable)
			request:yield()
		end
	end)

	scheduler:append_task(function()
		action:push_sentry_event(SentryEvent.SWITCH_POSE_DEFENSE)

		while true do
			if blackboard.user.health < 80 then
				action:push_sentry_event(SentryEvent.SWITCH_POSE_POWERED_DEFENSE)
			elseif blackboard.user.enemy_visible then
				action:push_sentry_event(SentryEvent.SWITCH_POSE_ATTACK)
			else
				action:push_sentry_event(SentryEvent.SWITCH_POSE_DEFENSE)
			end
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			while blackboard.game.stage ~= GameStage.STARTED or blackboard.user.health <= 0 do
				request:yield()
			end

			action:info("比赛开始，启动巡逻")
			action:switch_motion_mode("attack")
			action:gimbal_scan(0, 0)

			for _, point in ipairs(patrol_points) do
				action:navigate(point)

				local deadline = clock:now() + 5
				while clock:now() < deadline do
					if blackboard.user.enemy_visible then
						action:gimbal_toward(0, 0)
						while blackboard.user.enemy_visible do
							request:yield()
						end
						action:gimbal_scan(0, 0)
					end
					request:yield()
				end
			end
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
