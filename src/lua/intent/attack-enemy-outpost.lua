local action = require("action")
local Fsm = require("util.fsm")
local request = require("util.scheduler").request

local navigate_to = require("task.navigate-to")

return function()
	local State = { GO = "GO" }
	local TERMINAL = { x = 5, y = 5 }
	local DWELL = 5

	local fsm = Fsm:new(State.GO)

	fsm:use {
		state = State.GO,
		enter = function()
			action:info("attack-enemy-outpost: 进攻敌方前哨站")
			action:switch_motion_mode("attack")
		end,
		event = function(handle)
			navigate_to(TERMINAL, DWELL)
			handle:set_next(State.GO)
		end,
	}

	if not fsm:init_ready(State) then
		error("attack-enemy-outpost FSM 没有初始化完全")
	end

	while true do
		fsm:spin_once()
		request:yield()
	end
end
