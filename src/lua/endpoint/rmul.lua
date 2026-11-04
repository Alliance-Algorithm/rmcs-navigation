---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local Points = require("map.rmul").points

local task = {
	robot_status = require("task.robot_status"),
}

local kHealthLimit = 150 -- 低于该血量回家补给
local kBulletLimit = 20 -- 低于该弹量回家补给

local function remote_controller()
	while true do
		local rswitch = blackboard.play.rswitch
		action:update_enable_control(rswitch == "UP")

		request:yield()
	end
end

local function intent_maintainer()
	local Intent = {
		kNothing = "nothing",
		kCentral = "rmul-central",
		kSupply = "rmul-supply",
	}

	local handler = nil
	local runing_intent = Intent.kNothing
	local select_intent = runing_intent

	local last_stage = blackboard.game.stage

	while true do
		local stage = blackboard.game.stage
		if last_stage ~= GameStage.PREPARATION and stage == GameStage.PREPARATION then
			action:info("[Intent] 进入 PREPARATION 阶段")
			action:stop_navigation()

			select_intent = Intent.kNothing

			blackboard.context.runing_intent = Intent.kNothing
			blackboard.context.current = Points.kBegin
			blackboard.context.unhealth = false

			action:set_automatic_resurrection(true)
			action:update_track_building_only(false)
		end
		if last_stage == GameStage.PREPARATION and stage ~= GameStage.PREPARATION then
			action:info("[Intent] 退出 PREPARATION 阶段")
			action:restart_navigation {
				global_map = "rmul2027",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}
		end
		if last_stage ~= GameStage.STARTED and stage == GameStage.STARTED then
			action:info("[Intent] 进入 STARTED 阶段")
			select_intent = Intent.kCentral

			blackboard.context.started_timestamp = clock:now()
		end
		last_stage = stage

		-- 比赛阶段才执行的意图切换检测
		if stage == GameStage.STARTED then
			local health, bullet = blackboard.user.health, blackboard.user.bullet
			if (health < kHealthLimit or bullet < kBulletLimit) and runing_intent ~= Intent.kSupply then
				if select_intent ~= Intent.kSupply then
					action:info(string.format("[Intent] Supply, health: %d, bullet: %d", health, bullet))
				end
				select_intent = Intent.kSupply
				blackboard.context.unhealth = true
			elseif runing_intent == Intent.kSupply and not blackboard.context.unhealth then
				select_intent = Intent.kCentral
			end
		end

		-------

		-- 钻狗洞期间不切换意图，出洞后再切，避免云台停在折叠位、局部地图保持忽略
		if not blackboard.context.crossing then
			local need_append_task = false
			if (handler == nil) or handler.done() then
				need_append_task = true
			end
			if select_intent ~= runing_intent then
				if handler ~= nil then
					handler.cancel()
				end
				need_append_task = true
			end
			if need_append_task then
				local intent = require("intent." .. select_intent)
				handler = scheduler:append_task(intent.loop)

				runing_intent = select_intent
				blackboard.context.runing_intent = runing_intent
			end
		end
		request:yield()
	end
end

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	action:info(ascii.banner)
	action:info("Endpoint -> RMUL")

	scheduler:append_task(remote_controller)
	scheduler:append_task(intent_maintainer)
	scheduler:append_task(task.robot_status)

	action:bind(scheduler)
	clock:reset(blackboard.meta.timestamp)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function()
	action:stop_navigation()
end
