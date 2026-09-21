local MapRmuc = require("map.rmuc")
local Map, Points = MapRmuc.map, MapRmuc.points

return require("intent.patrol").new {
	map = Map,
	points = {
		Points.kOrigin, -- kBegin
		Points.kSelfHighlandBegin,
		Points.kAttackRune,
		Points.kAttackOutpost,
		Points.kSelfDoubleStepsBegin,
		Points.kSelfStepBegin,
		Points.kSelfStepFinal,
		Points.kSelfSlopeBegin,
		Points.kNearSelfOutpost,
		Points.kSelfDoubleStepsFinal,
		Points.kThemDoubleStepsFinal,
		Points.kNearThemOutpost,
		Points.kThemStepFinal,
		Points.kThemStepBegin,
		Points.kThemDoubleStepsBegin,
		Points.kThemHighlandBegin,
		Points.kThemHighlandFinal,
	},
	dwell = 3,
	timeout = 10,
	banner = "全图游走",
}
