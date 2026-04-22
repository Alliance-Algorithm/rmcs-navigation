local action = require("action")
local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request

--- Navigate to a waypoint and wait until success/interrupt/timeout.
--- @param args {
---   target: {x:number,y:number},
---   reach_tolerance: number,
---   waypoint_timeout: number,
---   is_stage_started: fun():boolean,
---   is_low_health: fun():boolean,
---   interrupt_on_low_health?: boolean,
---   on_timeout?: fun(target:{x:number,y:number})
--- }
--- @return boolean success
--- @return nil|"stage_not_started"|"low_health"|"timeout" reason
return function(args)
	local interrupt_on_low_health = args.interrupt_on_low_health
	if interrupt_on_low_health == nil then
		interrupt_on_low_health = true
	end

	action:navigate(args.target)
	local timeout = request:wait_until({
		monitor = function()
			local low_health_interrupt = interrupt_on_low_health and args.is_low_health()
			return blackboard.condition.near(args.target, args.reach_tolerance) or low_health_interrupt
				or (not args.is_stage_started())
		end,
		timeout = args.waypoint_timeout,
	})

	if not args.is_stage_started() then
		return false, "stage_not_started"
	end
	if interrupt_on_low_health and args.is_low_health() then
		return false, "low_health"
	end
	if timeout then
		if args.on_timeout ~= nil then
			args.on_timeout(args.target)
		end
		return false, "timeout"
	end
	return true, nil
end
