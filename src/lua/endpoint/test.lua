---
--- Local Context
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local order = require("util.order")
local edges = require("util.edge")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local test = 1

local restart_navigation = function()
	action:info("导航即将重启")
	action:restart_navigation {
		global_map = "empty",
		launch_livox = true,
		launch_odin1 = false,
		use_sim_time = false,
	}
	-- request:sleep(5)
	
	-- if test == 1 then
	    
    --     -- action:exchange_17mm_bullet(100)
	-- 	action:switch_mode(2)
	--     request:sleep(5)
    --     action:switch_mode(1)
	    
	--     request:sleep(3.2)

	--     test = 1
	-- end
    -- elseif test == 2 then
	--     action:switch_mode(2)

	--     request:sleep(3.2)

	--     -- action:exchange_17mm_bullet(200)
	--     -- request:sleep(3.2)

	--     test = 3
	-- elseif test == 3 then
	--     action:switch_mode(3)

	--     request:sleep(3.2)

	--     -- action:exchange_17mm_bullet(300)
	--     -- request:sleep(3.2)

	--     test = 1
	-- end
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
	action:switch_navigation(true)
	action:switch_topic_forward(true)

	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 0.5)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			restart_navigation()

			request:sleep(3)
			action:switch_mode(2)

		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		local _ = edges.new()

		while true do
			action:switch_navigation(blackboard.play.rswitch == "UP")
			request:yield()

			if blackboard.game.can_confirm_free_revive then
			    action:confirm_revive()
			end
		end
	end)

	scheduler:append_task(function()
		while true do
			request:sleep(1)
			action:info(string.format(
				"bullet %d health %d mode %d can_confirm_free_revive %s keyboard %d x %.2f y %.2f",
				blackboard.user.bullet,
				blackboard.user.health,
				blackboard.game.sentry_mode,
				tostring(blackboard.game.can_confirm_free_revive),
				blackboard.map_command.keyboard,
				blackboard.map_command.x,
				blackboard.map_command.y))
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	scheduler:spin_once()
end

on_exit = function() end
