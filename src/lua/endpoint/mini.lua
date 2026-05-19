---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local Fsm = require("util.fsm")
local option = require("option")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local task = {
	robot_status = require("task.robot_status"),
}

local intents = {
	escape_to_home = require("intent.escape-to-home"),
	attack_enemy_outpost = require("intent.attack-enemy-outpost"),
	guard_ally_outpost = require("intent.guard-ally-outpost"),
	patrol_fortress = require("intent.patrol-fortress"),
}

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	action:bind(scheduler)
	action:info("启用哨兵最小可执行方案")

	option:set_handler(function(error)
		action:fuck("while fetch option: " .. error)
	end)

	if option.enable_goal_topic_forward then
		action:switch_topic_forward(true)
	end

	action:info("启动导航")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
		launch_relocation = false,
	}

	scheduler:append_task(task.robot_status)

	--- 主决策协程: 重定位 → 等待开赛 → 顶层决策 FSM
	scheduler:append_task(function()
		--- 初始重定位,最多 3 次
		-- local relocalized = false
		-- for i = 1, 3 do
		-- 	action:info("初始化重定位第 " .. i .. " 次尝试")
		-- 	local ok = action:relocalize_initial(0, 0, 0)
		-- 	if ok then
		-- 		relocalized = true
		-- 		break
		-- 	end
		-- 	action:warn("初始化重定位失败,准备重试")
		-- end
		-- if not relocalized then
		-- 	action:fuck("初始化重定位三次都失败,决策流程终止")
		-- 	return
		-- end
		-- action:info("初始化重定位成功")

		--- 等待比赛进入 STARTED
		action:info("等待比赛进入 STARTED")
		request:wait_until {
			monitor = function()
				return blackboard.game.stage == "STARTED"
			end,
		}
		action:info("比赛已开始,进入决策循环")

		--- 顶层决策 FSM
		local Intent = {
			ESCAPE = "ESCAPE",
			ATTACK = "ATTACK",
			GUARD = "GUARD",
			PATROL = "PATROL",
		}

		local registry = {
			{ name = Intent.ESCAPE, log = "ESCAPE (低血/低弹)", run = intents.escape_to_home },
			{ name = Intent.ATTACK, log = "ATTACK (进攻敌方前哨站)", run = intents.attack_enemy_outpost },
			{ name = Intent.GUARD,  log = "GUARD (守卫己方前哨站)",  run = intents.guard_ally_outpost },
			{ name = Intent.PATROL, log = "PATROL (堡垒巡逻)",      run = intents.patrol_fortress },
		}

		local running = nil
		local function switch_to(entry)
			if running then
				running.cancel()
			end
			running = scheduler:append_task(entry.run)
		end

		local function decide()
			local cond = blackboard.condition
			if not cond.getout() then
				return Intent.ESCAPE
			elseif blackboard.outpost.them.health > 0 then
				return Intent.ATTACK
			elseif blackboard.outpost.ours.health > 0 then
				return Intent.GUARD
			else
				return Intent.PATROL
			end
		end

		local fsm = Fsm:new(decide())
		for _, entry in ipairs(registry) do
			fsm:use {
				state = entry.name,
				enter = function()
					action:info("decision: " .. entry.log)
					switch_to(entry)
				end,
				event = function(handle)
					handle:set_next(decide())
				end,
			}
		end
		if not fsm:init_ready(Intent) then
			error("决策 FSM 没有初始化完全")
		end

		while true do
			fsm:spin_once()
			request:yield()
		end
	end)

	action:info(ascii.banner)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function()
	action:stop_navigation()
end
