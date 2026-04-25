local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request
local action = require("action")

local POLL_INTERVAL = 0.2

--- 在补血点等待，直到血量达到 ready 阈值。
--- @return boolean is_success
return function()
	local condition = blackboard.condition
	local health_ready = blackboard.rule.health_ready
	assert(type(condition.health_ready) == "function", "blackboard.condition.health_ready should be a function")
	assert(type(health_ready) == "number", "blackboard.rule.health_ready should be a number")

	if condition.health_ready() then
		action:info(string.format(
			"supply-health: 当前已达到补血完成阈值 (health=%s, health_ready=%s)",
			tostring(blackboard.user.health),
			tostring(health_ready)
		))
		return true
	end

	action:info(string.format(
		"supply-health: 开始等待补血 (health=%s, health_ready=%s)",
		tostring(blackboard.user.health),
		tostring(health_ready)
	))

	while not condition.health_ready() do
		request:sleep(POLL_INTERVAL)
	end

	action:info(string.format(
		"supply-health: 血量已达到补血完成阈值，结束等待 (health=%s)",
		tostring(blackboard.user.health)
	))
	return true
end
