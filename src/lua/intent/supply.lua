local request = require("util.scheduler").request
local action = require("action")
local bb = require("blackboard").singleton()

local intent = {}

function intent:loop()
	local home = bb.rule.home

	action:info("需要补给，回家")
	action:info("health: " .. bb.user.health)
	action:info("bullet: " .. bb.user.bullet)

	action:navigate(home)

	local early_return = false
	while not bb.condition.near(home, 0.1) do
		local health = bb.user.health
		local bullet = bb.user.bullet
		if health >= bb.rule.health_ready and bullet >= bb.rule.bullet_ready then
			early_return = true
			break
		end
		request:sleep(1)
	end
	if not early_return then
		request:sleep(bb.rule.supply_interval)
	end

	bb.context.hint_intent = bb.context.intent_to_return
end

return intent
