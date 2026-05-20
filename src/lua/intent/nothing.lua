local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	while true do
		request:sleep(1)
	end
end

return intent
