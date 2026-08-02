---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local option = require("option")
local order = require("util.order")

local RmucMap = require("map.rmuc")
local Map, Points = RmucMap.map, RmucMap.points

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

local function remote_controller()
	local rswitch = blackboard.play.rswitch

	while true do
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
		kAttackRune = "attack-rune",
		kAttackOutpost = "attack-outpost",
	}

	local kHealthLimit = 200
	local kBulletLimit = 020

	local handler = nil
	local current = nil

	local last_stage = blackboard.game.stage

	local last_health = blackboard.user.health
	local last_bullet = blackboard.user.bullet

	while true do
		local select_intent = Intent.kNothing

		----

		local stage = blackboard.game.stage
		if last_stage ~= GameStage.PREPARATION and stage == GameStage.PREPARATION then
			action:info("[Intent] 进入 PREPARATION 阶段")
			action:stop_navigation()
			action:abort_record()
			select_intent = Intent.kNothing
		end
		if last_stage == GameStage.PREPARATION and stage ~= GameStage.PREPARATION then
			action:info("[Intent] 退出 PREPARATION 阶段")
			action:restart_navigation {
				global_map = "rmuc",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}
			action:start_record()

			-- action:relocalize()
		end
		if last_stage ~= GameStage.STARTED and stage == GameStage.STARTED then
			action:info("[Intent] 进入 STARTED 阶段")
			select_intent = Intent.kCruiseAtHome
		end
		last_stage = stage

		-- 比赛阶段才执行的意图切换检测
		if stage == GameStage.STARTED then
			if select_intent == Intent.kNothing then
				select_intent = Intent.kCruiseAtHome
			end

			local health, bullet = blackboard.user.health, blackboard.user.bullet
			local low_health = (health < kHealthLimit) and (last_health >= kHealthLimit)
			local low_bullet = (bullet < kBulletLimit) and (last_bullet >= kBulletLimit)
			if low_health or low_bullet then
				action:info(string.format("[Intent] Supply, health: %d, bullet: %d", health, bullet))
				select_intent = Intent.kSupply
			end
			last_health, last_bullet = health, bullet
		end

		----

		local need_append_task = false
		if (handler == nil) or handler.done() then
			need_append_task = true
		end
		if select_intent ~= current then
			if handler ~= nil then
				handler.cancel()
			end
			need_append_task = true
		end
		if need_append_task then
			local intent = require("intent." .. select_intent)
			handler = scheduler:append_task(intent.loop)

			current = select_intent
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

	_ = Map
	_ = Points

	scheduler:append_task(remote_controller)
	scheduler:append_task(intent_maintainer)
	scheduler:append_task(task.robot_status)

	blackboard.context.current = Points.kOrigin -- 初始出生点

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
