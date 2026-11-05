local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")

local MapRmul = require("map.rmul")
local Points = MapRmul.points

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local kTasks = {
	robot_status = require("task.robot_status"),
}

local kHealthLimit = 150
local kBulletLimit = 20
local kFakeBullet = 300

local Intent = {
	kCentral = "rmul-central",
	kSupply = "rmul-supply",
}

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:bind(scheduler)
	action:info(ascii.banner)
	action:info("RMUL 策略测试......")

	clock:reset(blackboard.meta.timestamp)
	action:update_enable_control(false)

	local train_started = false
	local intent_handler = nil
	local running_intent = nil
	local select_intent = nil

	local function stop_intent()
		if intent_handler ~= nil then
			intent_handler.cancel()
			intent_handler = nil
		end
		running_intent = nil
		select_intent = nil
		blackboard.context.runing_intent = "nothing"
		blackboard.context.crossing = false
		blackboard.context.engage_on_route = false
		blackboard.context.unhealth = false
	end

	local toggle_train = function()
		if train_started then
			action:info("导航关闭")
			train_started = false

			action:switch_motion_mode("normal")
			action:gimbal_suspend()

			request:sleep(0.5)
			action:update_enable_control(false)
			action:switch_topic_forward(false)

			action:update_supercap_boost(false)
			action:set_local_map_ignored(false, 0, 1.0)
			action:blocking_set_gimbal_fold(false)
			action:stop_navigation()

			stop_intent()
		else
			action:info("导航启动")

			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			request:sleep(5)
			-- action:relocalize()
			-- request:sleep(2)

			action:update_enable_control(true)
			action:switch_topic_forward(true)
			action:gimbal_suspend()

			blackboard.context.current = Points.kBegin
			blackboard.context.crossing = false
			blackboard.context.engage_on_route = false
			blackboard.context.unhealth = false
			select_intent = Intent.kCentral
			train_started = true
		end
	end

	scheduler:append_task(function()
		while true do
			if train_started then
				local health = blackboard.user.health
				local bullet = kFakeBullet
				if (health < kHealthLimit or bullet < kBulletLimit) and running_intent ~= Intent.kSupply then
					if select_intent ~= Intent.kSupply then
						action:info(string.format("[Intent] Supply, health: %d, bullet: %d", health, bullet))
					end
					select_intent = Intent.kSupply
					blackboard.context.unhealth = true
					action:switch_motion_mode("normal")
				elseif running_intent == Intent.kSupply and not blackboard.context.unhealth then
					select_intent = Intent.kCentral
				end

				if not blackboard.context.crossing and select_intent ~= nil then
					local need_append_task = intent_handler == nil or intent_handler.done()
					if select_intent ~= running_intent then
						if intent_handler ~= nil then
							intent_handler.cancel()
						end
						need_append_task = true
					end
					if need_append_task then
						local intent = require("intent." .. select_intent)
						intent_handler = scheduler:append_task(intent.loop)
						running_intent = select_intent
						blackboard.context.runing_intent = running_intent
					end
				end
			end
			request:yield()
		end
	end)
	scheduler:append_task(kTasks.robot_status)
	scheduler:append_task(function()
		while true do
			local position = action:current_position()
			action:info(string.format("当前坐标: x=%.3f, y=%.3f", position.x, position.y))
			request:sleep(1)
		end
	end)
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 1)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, toggle_train)

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
	action:stop_navigation()
end
