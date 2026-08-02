local action = require("action")
local api = require("api")
local request = require("util.scheduler").request
local bb = require("blackboard").singleton()

local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

local intent = {}

local kNavigateTimeout = 10
local kScanTimeout = 20

function intent:loop()
	action:info("Attack rune start")
	action:gimbal_toward(0, 0)
	action:navigate(Points.kAttackRune)
	local navigate_timeout = request:wait_until {
		monitor = function()
			return bb.condition.near(Points.kAttackRune, 0.5)
		end,
		timeout = kNavigateTimeout,
	}
	if navigate_timeout then
		action:warn("navigate to rune timeout")
		blackboard.context.attacked_rune = true
		return
	end
	bb.context.current = Points.kAttackRune

	api.sentry_event(SentryEvent.ACTIVATE_ENERGY_CORE)
	api.update_track_rune(true)
	action:gimbal_scan(-6 * math.pi / 12, -math.pi / 12)
	action:switch_motion_mode("attack")

	local scan_timeout = request:wait_until {
		monitor = function()
			return bb.energy.small == 1 or bb.energy.big == 1
		end,
		timeout = kScanTimeout,
	}

	api.update_track_rune(false)
	action:info(scan_timeout and "attack rune timeout" or "rune activated")

	blackboard.context.attacked_rune = true
end

return intent
