local function create_default_blackboard()
	return {
		user = {
			health = 0,
			bullet = 0,
		},
		game = {
			stage = "UNKNOWN",
		},
		rule = {
			health_limit = 0,
			health_ready = 0,
			bullet_limit = 0,
			bullet_ready = 0,
		},
		play = {
			rswitch = "UNKNOWN",
			lswitch = "UNKNOWN",
		},
		meta = {
			timestamp = 0,
		},
	}
end
local blackboard_singleton = create_default_blackboard()

local BlackboardDetails = {}
function BlackboardDetails.get_blackboard()
	return blackboard_singleton
end

return BlackboardDetails
