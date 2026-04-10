---
--- Local Context
---

local api = require("api")
local clock = require("util.clock")
local fsm = require("util.fsm")

local blackboard_module = require("blackboard")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local edges = require("util.edge").new()

local NaN = 0 / 0
local patrol_goals = {
	{ x = 2.0, y = 0.0 },
	{ x = 0.0, y = 0.0 },
}
local cache = {
	goal = { x = NaN, y = NaN },
}
function cache:send_goal()
	local x = self.goal.x
	local y = self.goal.y
	if x ~= x or y ~= y then
		return
	end
	api.apply_navigation_goal(x, y)
end

local function set_goal(goal)
	cache.goal.x = goal.x
	cache.goal.y = goal.y
end

---
--- Export Context
---

blackboard = blackboard_module.singleton()

on_init = function()
	clock:reset(blackboard.meta.timestamp)

	scheduler:append_task(function()
		local goal_index = 1
		while true do
			request:wait_until {
				monitor = function()
					return blackboard.getter.rswitch() == "UP"
				end,
			}

			goal_index = 1
			while blackboard.getter.rswitch() == "UP" do
				set_goal(patrol_goals[goal_index])
				cache:send_goal()
				goal_index = goal_index % #patrol_goals + 1
				request:sleep(5)
			end
		end
	end)

	-- 立即响应导航点的切换
	-- scheduler:append_task(function()
	-- 	local last = { x = cache.goal.x, y = cache.goal.y }
	-- 	while true do
	-- 		local x = cache.goal.x
	-- 		local y = cache.goal.y

	-- 		if x ~= last.x or y ~= last.y then
	-- 			cache:send_goal()
	-- 		end

	-- 		last = { x = x, y = y }
	-- 		request:yield()
	-- 	end1
	-- end)

	-- scheduler:append_task(function()
	-- 	--- @enum Motion
	-- 	local Motion = {
	-- 		IDLE = "IDLE",
	-- 		FREE = "FREE",
	-- 		SPIN = "SPIN",
	-- 	}
	-- 	local motion = fsm:new(Motion.FREE)

	-- 	motion:use {
	-- 		state = Motion.FREE,
	-- 		enter = function()
	-- 			api.info("Enter Motion::FREE")
	-- 		end,
	-- 		event = function(handle)
	-- 			handle:set_next(Motion.IDLE)
	-- 		end,
	-- 	}

	-- 	-- @TODO:
	-- 	--  继续开发运动状态机

	-- 	if not motion:init_ready(Motion) then
	-- 		error("Failed to init Motion Fsm, need all state be used")
	-- 	end

	-- 	while true do
	-- 		motion:spin_once()
	-- 		request:yield()
	-- 	end
	-- end)

	edges:on(blackboard.getter.rswitch, "UP", function()
		api.restart_navigation("rmul")
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)

	edges:spin()
	scheduler:spin_once()
end

--- 由 NAV2 发布的目标速度值，在此处理回调
control_speed_callback = function(vx, vy, qx)
	local _ = qx
	api.update_chassis_vel(vx, vy)
end
