local request = require("util.scheduler").request
local follow_waypoints = require("task.follow-waypoints")

--- Loop through cruise waypoints forever until interrupted.
--- @param args {
---   points: { {x:number,y:number}[] },
---   navigate: fun(point:{x:number,y:number}, args?:{interrupt_on_low_health:boolean}):(boolean,nil|string),
---   retry_delay?: number,
---   idle_sleep?: number
--- }
--- @return boolean completed
--- @return nil|"stage_not_started"|"low_health"|"empty" reason
return function(args)
	local points = args.points or {}
	if #points == 0 then
		request:sleep(args.idle_sleep or 0.5)
		return false, "empty"
	end

	while true do
		local success, reason = follow_waypoints({
			points = points,
			navigate = args.navigate,
			interrupt_on_low_health = true,
			advance_on_timeout = true,
			retry_delay = args.retry_delay or 0.1,
		})
		if not success then
			return false, reason
		end
	end
end
