local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

return require("intent.patrol").new {
	map = Map,
	points = {
		Points.kThemDoubleStepsBegin,
		Points.kThemThigh,
		Points.kThemHighlandBegin,
	},
	dwell = 5,
	timeout = 10,
	banner = "在对方半场巡逻（双台阶起点/大腿/高地起点），每 5s 切换一次",
}
