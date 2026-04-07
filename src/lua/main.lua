---
--- Context
---

local api = require("api")
local option = require("option")

local Bt = require("util.behavior")
local clock = require("util.clock")
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

local NaN = 0 / 0
local cache = {
	goal = { x = NaN, y = NaN },
}
local function apply_navigation_goal()
	local x = cache.goal.x
	local y = cache.goal.y
	if x ~= x or y ~= y then
		return
	end
	api.apply_navigation_goal(x, y)
end

---
--- Export Function
---

--- @export
--- 由 NAV2 发布的目标速度值，在此处理回调
function control_speed_callback(vx, vy, qx)
	local _ = qx
	api.update_chassis_vel(vx, vy)
end

--- @export
function on_init()
	clock:reset(blackboard.meta.timestamp)

	behavior:bind(context)

	interrupt:on(blackboard.condition.low_health)
	interrupt:on(blackboard.condition.low_bullet)

	-- 定期更新导航的目标，防止规划失败后停滞
	context:spawn(function(handle)
		while true do
			handle:sleep(2.0)
			apply_navigation_goal()
		end
	end)

	-- 立即响应导航点的切换
	context:spawn(function(handle)
		local last = { x = cache.goal.x, y = cache.goal.y }
		while true do
			local x = cache.goal.x
			local y = cache.goal.y

			if x ~= last.x or y ~= last.y then
				apply_navigation_goal()
			end

			last.x = x
			last.y = y
			handle:yield()
		end
	end)

	edges:on(blackboard.getter.rswitch, "UP", function()
		api.restart_navigation("rmul")
	end)
end

--- @export
function on_tick()
	clock:update(blackboard.meta.timestamp)

	edges:spin()
	interrupt:spin()
	context:spin(blackboard.meta.timestamp)
end
