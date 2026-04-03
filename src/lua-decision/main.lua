local api = require("api")
local option = require("option")

local Bt = require("lib.behavior")
local Interrupt = require("lib.interrupt")
local blackboard = require("blackboard").singleton()

local decision_name = blackboard.rule.decision
local decision = option.decisions[decision_name]
if not decision then
	error("unknown decision: " .. tostring(decision_name))
end

local edges = require("lib.edge").edges()
local context = require("lib.io_context").new()
local behavior = Bt.new(decision)
local interrupt = Interrupt.new(behavior)

---@export
function on_init()
	behavior:bind(context)

	interrupt:on(blackboard.condition.low_health)
	interrupt:on(blackboard.condition.low_bullet)

	edges:on(blackboard.getter.rswitch, "UP", function()
		api.restart_navigation("rmul")
	end)
end

---@export
function on_tick()
	edges:spin()
	context:spin(blackboard.meta.timestamp)
	interrupt:spin()
end
