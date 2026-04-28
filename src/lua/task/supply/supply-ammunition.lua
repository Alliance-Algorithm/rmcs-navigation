local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request
local action = require("action")

local POLL_INTERVAL = 0.2

--- 在补给点等待，直到弹量达到 ready 阈值。
--- @return boolean is_success
return function()
	local condition = blackboard.condition
	local bullet_ready = blackboard.rule.bullet_ready
	assert(type(condition.bullet_ready) == "function", "blackboard.condition.bullet_ready should be a function")
	assert(type(bullet_ready) == "number", "blackboard.rule.bullet_ready should be a number")

	if condition.bullet_ready() then
		action:info(string.format(
			"supply-ammunition: 当前已达到补弹完成阈值 (bullet=%s, bullet_ready=%s)",
			tostring(blackboard.user.bullet),
			tostring(bullet_ready)
		))
		return true
	end

	action:info(string.format(
		"supply-ammunition: 开始等待补弹 (bullet=%s, bullet_ready=%s)",
		tostring(blackboard.user.bullet),
		tostring(bullet_ready)
	))

	while not condition.bullet_ready() do
		request:sleep(POLL_INTERVAL)
	end

	action:info(string.format(
		"supply-ammunition: 弹量已达到补弹完成阈值，结束等待 (bullet=%s)",
		tostring(blackboard.user.bullet)
	))
	return true
end
