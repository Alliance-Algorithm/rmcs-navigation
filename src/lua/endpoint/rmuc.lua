---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local order = require("util.order")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

local function remote_controller()
	while true do
		local rswitch = blackboard.play.rswitch

		local enable_navigation = (rswitch == "UP")
		action:update_enable_control(enable_navigation)

		request:yield()
	end
end

local function intent_maintainer()
	local Intent = {
		kNothing = "nothing",
		kSupply = "supply",
		kCruiseAtHome = "cruise-at-home",
		kCruiseAtHighland = "cruise-at-highland",
		kCruiseAtFullMap = "cruise-at-full-map",
		kAttackRune = "attack-rune",
		kAttackOutpost = "attack-outpost",
	}

	local main_intents = {
		Intent.kAttackRune,
		Intent.kAttackOutpost,
		Intent.kCruiseAtHome,
	}
	local function remove_from_main_intents(name)
		for i, item in ipairs(main_intents) do
			if item == name then
				table.remove(main_intents, i)
				break
			end
		end
	end
	local function reset_main_intents()
		main_intents = {
			Intent.kAttackRune,
			Intent.kAttackOutpost,
			Intent.kCruiseAtHome,
		}
	end

	local handler = nil
	local runing_intent = Intent.kNothing
	local select_intent = runing_intent

	local kHealthLimit = blackboard.rule.health_limit
	local kBulletLimit = blackboard.rule.bullet_limit

	local last_stage = blackboard.game.stage

	while true do
		local stage = blackboard.game.stage
		if last_stage ~= GameStage.PREPARATION and stage == GameStage.PREPARATION then
			action:info("[Intent] 进入 PREPARATION 阶段")
			action:stop_navigation()
			action:abort_record()
			select_intent = Intent.kNothing

			reset_main_intents()
			blackboard.context.attacked_outpost = false
			blackboard.context.attacked_rune = false
		end
		if last_stage == GameStage.PREPARATION and stage ~= GameStage.PREPARATION then
			action:info("[Intent] 退出 PREPARATION 阶段")
			action:restart_navigation {
				global_map = "empty",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}
			action:start_record()

			-- action:relocalize()
		end
		if last_stage ~= GameStage.STARTED and stage == GameStage.STARTED then
			action:info("[Intent] 进入 STARTED 阶段")
			select_intent = main_intents[1]

			blackboard.context.started_timestamp = clock:now()
		end
		last_stage = stage

		-- 比赛阶段才执行的意图切换检测
		if stage == GameStage.STARTED then
			local health, bullet = blackboard.user.health, blackboard.user.bullet
			local low_health = (health < kHealthLimit)
			local low_bullet = (bullet < kBulletLimit)
			if (low_health or low_bullet) and runing_intent ~= Intent.kSupply then
				action:info(string.format("[Intent] Supply, health: %d, bullet: %d", health, bullet))
				select_intent = Intent.kSupply
				blackboard.context.unhealth = true
				goto FINALIZED
			end

			if (runing_intent == Intent.kSupply) and not blackboard.context.unhealth then
				select_intent = main_intents[1]
				goto FINALIZED
			end
			if
				(runing_intent == Intent.kAttackOutpost and blackboard.context.attacked_outpost)
				or (runing_intent == Intent.kAttackRune and blackboard.context.attacked_rune)
			then
				remove_from_main_intents(runing_intent)
				select_intent = main_intents[1]

				goto FINALIZED
			end

			::FINALIZED::
			if select_intent == Intent.kNothing then
				-- Unreachable
				select_intent = main_intents[1]
			end
		end

		-------

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
	action:info("Endpoint -> RMUC")

	_ = fsm
	_ = option
	_ = order
	_ = task

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
