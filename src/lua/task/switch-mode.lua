local action = require("action")

--- @param mode string
--- @return boolean is_success
return function(mode)
	assert(type(mode) == "string", "mode should be a string")
	action:update_chassis_mode(mode)
	return true
end
