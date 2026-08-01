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

---
--- Export Context
---

blackboard = require("blackboard").singleton()

on_init = function()
	_ = ascii
	_ = fsm
	_ = option
	_ = order
	_ = task
	_ = request

	action:bind(scheduler)
	clock:reset(blackboard.meta.timestamp)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	scheduler:spin_once()
end

on_exit = function() end
