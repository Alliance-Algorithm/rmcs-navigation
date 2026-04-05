local api = require("api")
local option = require("option")

local Bt = require("util.behavior")
local Interrupt = require("util.interrupt")
local Blackboard = require("blackboard")
local blackboard = Blackboard.singleton()

local decision_name = blackboard.rule.decision
local decision = option.decisions[decision_name]
if not decision then
	error("unknown decision: " .. tostring(decision_name))
end

local edges = require("util.edge").edges()
local context = require("util.io_context").new()
local behavior = Bt.new(decision)
local interrupt = Interrupt.new(behavior)

function on_init()
	behavior:bind(context)

	interrupt:on(blackboard.condition.low_health)
	interrupt:on(blackboard.condition.low_bullet)

	edges:on(blackboard.getter.rswitch, "UP", function()
		api.restart_navigation("rmul")
	end)
end

function on_tick()
	edges:spin()
	interrupt:spin()
	context:spin(blackboard.meta.timestamp)
end
