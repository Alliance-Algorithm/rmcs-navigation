local TrainMap = require("train_map")

local M = {}

local function select_rule_point(point, ours_zone)
	if type(point.x) == "number" and type(point.y) == "number" then
		return point
	end
	return ours_zone and point.ours or point.them
end

local function distance(a, b)
	local dx = a.x - b.x
	local dy = a.y - b.y
	return math.sqrt(dx * dx + dy * dy)
end

local function nearest_road_return_route(position, rule, ours_zone)
	local candidates = {
		{
			route = "road_region_final",
			point = select_rule_point(rule.road_zone_final, ours_zone),
		},
		{
			route = "road_region_2",
			point = select_rule_point(rule.road_zone_way_point_2, ours_zone),
		},
		{
			route = "road_region_1",
			point = select_rule_point(rule.road_zone_way_point_1, ours_zone),
		},
		{
			route = "road_region_begin",
			point = select_rule_point(rule.road_zone_begin, ours_zone),
		},
	}

	local selected = candidates[1]
	local selected_distance = distance(position, selected.point)
	for index = 2, #candidates do
		local candidate = candidates[index]
		local candidate_distance = distance(position, candidate.point)
		if candidate_distance < selected_distance then
			selected = candidate
			selected_distance = candidate_distance
		end
	end

	return selected.route
end

function M.select(args)
	local Region = TrainMap.Region

	if args.region == Region.OURS_HOME then
		return "ours_home"
	end

	if args.region == Region.ROAD_REGION_BEGIN then
		return "road_region_begin"
	end

	if args.region == Region.ROAD_REGION_1 then
		return "road_region_1"
	end

	if args.region == Region.ROAD_REGION_2 then
		return "road_region_2"
	end

	if args.region == Region.ROAD_REGION_FINAL then
		return "road_region_final"
	end

	if args.region == Region.OURS_HIGHLAND then
		return "highland"
	end

	return nearest_road_return_route(args.position, args.rule, args.ours_zone)
end

return M
