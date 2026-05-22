local action = require("action")
local request = require("util.scheduler").request

local intent = {}

function intent:loop()
	action:info("进入到什么也不做的意图")
	action:switch_motion_mode("normal")

	while true do
		action:info("间隔非常长的心跳，以防你不知道哨兵还活着")
		request:sleep(10)
	end
end

return intent
