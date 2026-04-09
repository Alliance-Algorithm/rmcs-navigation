local function create_default_blackboard()
	local bb = {
		user = {
			health = 0,
			bullet = 0,
			x = 0,
			y = 0,
		},
		game = {
			stage = "UNKNOWN",
		},
		play = {
			rswitch = "UNKNOWN",
			lswitch = "UNKNOWN",
		},
		rule = {
			decision = "auxiliary",

			health_limit = 0,
			health_ready = 0,
			bullet_limit = 0,
			bullet_ready = 0,

			home = { 0., 0. },
		},
		meta = {
			timestamp = 0,
		},
	}

	bb.condition = {
		low_health = function()
			return bb.user.health < bb.rule.health_limit
		end,
		low_bullet = function()
			return bb.user.bullet < bb.rule.bullet_limit
		end,
		health_ready = function()
			return bb.user.health >= bb.rule.health_ready
		end,
		bullet_ready = function()
			return bb.user.bullet >= bb.rule.bullet_ready
		end,
	}

	bb.getter = {
		rswitch = function()
			return bb.play.rswitch
		end,
	}

	return bb
end

local blackboard_singleton = create_default_blackboard()

local BlackboardDetails = {}
function BlackboardDetails.singleton()
	return blackboard_singleton
end

return BlackboardDetails
