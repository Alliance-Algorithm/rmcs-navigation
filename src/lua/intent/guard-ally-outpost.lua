local action = require("action")
local Fsm = require("util.fsm")
local request = require("util.scheduler").request

local navigate_to = require("task.navigate-to")

return function()
	local State = { P1 = "P1", P2 = "P2" }
	local DWELL = 5

	local legs = {
		[State.P1] = { point = { x = 2, y = 2 }, next = State.P2 },
		[State.P2] = { point = { x = 3, y = 0 }, next = State.P1 },
	}

	local fsm = Fsm:new(State.P1)

	for state, leg in pairs(legs) do
		fsm:use {
			state = state,
			enter = function()
				action:info("guard-ally-outpost: 巡逻到 " .. state)
				action:switch_motion_mode("attack")
			end,
			event = function(handle)
				navigate_to(leg.point, DWELL)
				handle:set_next(leg.next)
			end,
		}
	end

	if not fsm:init_ready(State) then
		error("guard-ally-outpost FSM 没有初始化完全")
	end

	while true do
		fsm:spin_once()
		request:yield()
	end
end
