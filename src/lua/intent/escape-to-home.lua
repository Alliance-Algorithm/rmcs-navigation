local action = require("action")
local blackboard = require("blackboard").singleton()
local Fsm = require("util.fsm")
local request = require("util.scheduler").request

local navigate_to = require("task.navigate-to")

return function()
	local State = { SUPPLY = "SUPPLY" }
	local HOME = blackboard.rule.resupply_zone.ours
	local DWELL = 5

	local fsm = Fsm:new(State.SUPPLY)

	fsm:use {
		state = State.SUPPLY,
		enter = function()
			action:info("escape-to-home: 回补给区")
			action:switch_motion_mode("normal")
		end,
		event = function(handle)
			navigate_to(HOME, DWELL)
			handle:set_next(State.SUPPLY)
		end,
	}

	if not fsm:init_ready(State) then
		error("escape-to-home FSM 没有初始化完全")
	end

	while true do
		fsm:spin_once()
		request:yield()
	end
end
