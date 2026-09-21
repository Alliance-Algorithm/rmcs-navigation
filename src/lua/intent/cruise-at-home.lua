local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

return require("intent.patrol").new {
	map = Map,
	points = {
		Points.kBehindOutpost,
		Points.kSelfStepBegin,
		Points.kSelfDoubleStepsBegin,
		Points.kSelfHighlandBegin,
		Points.kAttackRune,
		Points.kAttackOutpost,
		Points.kAttackRune,
		Points.kSelfHighlandBegin,
		Points.kSelfDoubleStepsBegin,
		Points.kSelfStepBegin,
	},
	dwell = 5,
	timeout = 10,
	resume = true,
	banner = "在家中巡游，形如地刺",
}
