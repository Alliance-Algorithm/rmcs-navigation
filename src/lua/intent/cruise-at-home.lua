local action = require("action")
local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	action:info("在家中巡游，形如地刺")
	action:switch_motion_mode("attack")

	while true do
		request:sleep(1)
	end
end

return intent
