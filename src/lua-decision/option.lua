local bt = require("util.behavior")
local bb = require("blackboard").singleton()
local api = require("api")

local actions = {
	return_home = bt.action(function(ctx)
		api.update_goal(bb.rule.home)
		ctx:wait_until(bb.condition.health_ready)
		return bt.SUCCESS
	end),
}

local sequences = {
	low_health = bt.sequence({
		bt.condition(bb.condition.low_health),
		actions.return_home,
	}),
	low_bullet = bt.sequence({
		bt.condition(bb.condition.low_bullet),
		actions.return_home,
	}),
}

return {
	decisions = {
		auxiliary = bt.selector({
			sequences.low_health,
			sequences.low_bullet,

			bt.action(function(_)
				return bt.RUNNING
			end),
		}),
	},
}
