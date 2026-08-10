local action = require("action")
local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	action:info("Waiting......")
	action:switch_motion_mode("normal")

	action:gimbal_free()

	while true do
		action:info(".")
		request:sleep(30)
	end
end

return intent
