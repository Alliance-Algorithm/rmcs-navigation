---
--- Local Context
---

local enable_automatic_record_lidar = false

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

		action:update_enable_control(rswitch == "UP")

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
		kAttackBase = "attack-base",
		kAttackOutpost = "attack-outpost",
		kkCruiseAtThemHome = "cruise-at-them-home",
	}

	local Command = { -- 操作手 UI 地图坐标系，非机器人坐标，需注意
		kBase = {
			{ x = 2.50, y = 7.5 },
			{ x = 25.5, y = 7.5 },
		},
		kHelipad = {
			{ x = 1.37, y = 13.17 },
			{ x = 26.6, y = 2.03 },
		},
	}
	local near = function(ax, ay, points)
		return (math.abs(ax - points[1].x) < 1 and math.abs(ay - points[1].y) < 1)
			or (math.abs(ax - points[2].x) < 1 and math.abs(ay - points[2].y) < 1)
	end

	local main_intents = {}
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
			Intent.kAttackOutpost,
			-- Intent.kCruiseAtHighland,
			Intent.kkCruiseAtThemHome,
		}
	end
	reset_main_intents()

	local handler = nil
	local runing_intent = Intent.kNothing
	local select_intent = runing_intent

	local forced_enable = false
	local forced_intent = Intent.kAttackBase

	local kHealthLimit = blackboard.rule.health_limit
	local kBulletLimit = blackboard.rule.bullet_limit

	local last_stage = blackboard.game.stage

	local last_command_x = blackboard.map_command.x
	local last_command_y = blackboard.map_command.y

	while true do
		local stage = blackboard.game.stage
		if last_stage ~= GameStage.PREPARATION and stage == GameStage.PREPARATION then
			action:info("[Intent] 进入 PREPARATION 阶段")
			action:stop_navigation()

			if enable_automatic_record_lidar then
				action:abort_record()
			end

			select_intent = Intent.kNothing
			forced_enable = false
			blackboard.context.runing_intent = Intent.kNothing

			reset_main_intents()
			blackboard.context.attacked_outpost = false
			blackboard.context.attacked_rune = false
			action:set_automatic_resurrection(true)
		end
		if last_stage == GameStage.PREPARATION and stage ~= GameStage.PREPARATION then
			action:info("[Intent] 退出 PREPARATION 阶段")
			action:restart_navigation {
				global_map = "rmuc",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
			}

			if enable_automatic_record_lidar then
				request:sleep(2)
				action:start_record()
			end

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
			-- [] Map Command，优先级最高
			local command_x = blackboard.map_command.x
			local command_y = blackboard.map_command.y
			if last_command_x ~= command_x or last_command_y ~= command_y then
				action:warn("[Intent] 目标点更新: (" .. command_x .. ", " .. command_y .. ")")

				if near(command_x, command_y, Command.kBase) then
					action:info("[Intent] 接收云台手指令，开始进攻对方基地")
					action:set_automatic_resurrection(false)
					forced_enable = true
					forced_intent = Intent.kAttackBase
				elseif near(command_x, command_y, Command.kHelipad) then
					action:info("[Intent] 接收云台手指令，恢复正常意图")
					action:set_automatic_resurrection(true)
					forced_enable = false
					select_intent = main_intents[1]
				end
			end
			last_command_x = command_x
			last_command_y = command_y

			-- [] Health And Bullet
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

			-- [] Consume Main Intents
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

			if forced_enable then
				select_intent = forced_intent
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
			blackboard.context.runing_intent = runing_intent
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
