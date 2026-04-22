local request = require("util.scheduler").request

--- Execute waypoints once.
--- @param args {
---   points: { {x:number,y:number}[] },
---   navigate: fun(point:{x:number,y:number}, args?:{interrupt_on_low_health:boolean}):(boolean,nil|string),
---   interrupt_on_low_health?: boolean,
---   advance_on_timeout?: boolean,
---   retry_delay?: number,
---   on_before_point?: fun(point:{x:number,y:number}, index:number, total:number),
---   on_reached?: fun(point:{x:number,y:number}, index:number, total:number)
--- }
--- @return boolean completed
--- @return nil|"stage_not_started"|"low_health"|"timeout" reason
return function(args)
	local points = args.points or {}
	local interrupt_on_low_health = args.interrupt_on_low_health
	if interrupt_on_low_health == nil then
		interrupt_on_low_health = true
	end

	local advance_on_timeout = args.advance_on_timeout
	if advance_on_timeout == nil then
		advance_on_timeout = true
	end

	local retry_delay = args.retry_delay or 0.1
	local index = 1
	while index <= #points do
		local point = points[index]

		if args.on_before_point ~= nil then
			args.on_before_point(point, index, #points)
		end

		local success, reason = args.navigate(point, {
			interrupt_on_low_health = interrupt_on_low_health,
		})
		if reason == "stage_not_started" or reason == "low_health" then
			return false, reason
		end

		if success then
			if args.on_reached ~= nil then
				args.on_reached(point, index, #points)
			end
			index = index + 1
		elseif reason == "timeout" then
			if advance_on_timeout then
				index = index + 1
			else
				request:sleep(retry_delay)
			end
		else
			request:sleep(retry_delay)
		end
	end

	return true, nil
end
