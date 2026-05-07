local M = {}

function M.configure(rule, read_option)
	rule.health_limit = read_option("health_limit", rule.health_limit)
	rule.health_ready = read_option("health_ready", rule.health_ready)
	rule.bullet_limit = read_option("bullet_limit", rule.bullet_limit)
	rule.bullet_ready = read_option("bullet_ready", rule.bullet_ready)

	rule.resupply_zone.ours = { x = 0.0, y = 0.0 }
	rule.road_zone_begin.ours = { x = 3.9, y = -0.8 }
	rule.road_zone_final.ours = { x = 4.9, y = -2.6 }
	rule.road_zone_way_point_1.ours = { x = 1.3, y = -1.2 }
	rule.road_zone_way_point_2.ours = { x = 1.3, y = -2.5 }
	rule.central_highland_middle.ours = { x = 5.4, y = 1.7 }
	rule.central_highland_near_fluctuant_road.ours = { x = 5.8, y = -0.6 }
	rule.central_highland_near_doghole.ours = { x = 6.2, y = 3.7 }
end

return M
