local blackboard = require("blackboard").singleton()
local Map = require("map")
local ReturnStage = require("util.return-stage")
local RegionId = Map.Region

local Phase = {
	unknown = "unknown",
	before_fluctuant = "before_fluctuant",
	on_fluctuant = "on_fluctuant",
	after_fluctuant = "after_fluctuant",
}

local M = {
	Phase = Phase,
}

function M.current()
	local map = Map.singleton()
	local region = map:locate({
		x = blackboard.user.x,
		y = blackboard.user.y,
	})
	return region, map.names[region] or "unknown"
end

function M.phase(region)
	if region == RegionId.OURS_HOME
		or region == RegionId.THEM_HOME
		or region == RegionId.OURS_ROAD_TO_FLUCTUANT
		or region == RegionId.THEM_ROAD_TO_FLUCTUANT then
		return Phase.before_fluctuant
	end

	if region == RegionId.OURS_FLUCTUANT or region == RegionId.THEM_FLUCTUANT then
		return Phase.on_fluctuant
	end

	if region == RegionId.OURS_TRAPEZOIDAL_HIGHLAND
		or region == RegionId.THEM_TRAPEZOIDAL_HIGHLAND
		or region == RegionId.OURS_ROAD_TO_HIGHLAND
		or region == RegionId.THEM_ROAD_TO_HIGHLAND
		or region == RegionId.OURS_HIGHLAND
		or region == RegionId.THEM_HIGHLAND then
		return Phase.after_fluctuant
	end

	return Phase.unknown
end

function M.return_stage(region)
	local phase = M.phase(region)
	if phase == Phase.on_fluctuant then
		return ReturnStage.on_fluctuant
	end
	if phase == Phase.after_fluctuant then
		return ReturnStage.after_fluctuant
	end
	return ReturnStage.before_fluctuant
end

function M.escape_route(region)
	return ReturnStage.resolve_escape_route(M.return_stage(region))
end

function M.is_before_fluctuant(region)
	return M.phase(region) == Phase.before_fluctuant
end

function M.is_on_fluctuant(region)
	return M.phase(region) == Phase.on_fluctuant
end

function M.is_after_fluctuant(region)
	return M.phase(region) == Phase.after_fluctuant
end

return M
