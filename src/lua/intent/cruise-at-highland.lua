local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

return require("intent.patrol").new {
	map = Map,
	points = {
		Points.kNearThemOutpost,
		Points.kThemDoubleStepsFinal,
	},
	dwell = 5,
	timeout = 20,
	banner = "在对方前哨与双台阶终点之间巡逻，每 5s 切换一次",
}
