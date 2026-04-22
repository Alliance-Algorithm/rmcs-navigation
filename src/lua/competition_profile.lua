-- Competition mission profile.
-- Edit this file to maintain route points inside rmcs-navigation-MAIN.

local profile = {
	-- Mission gate.
	start_stage = "STARTED",
	health_limit = 120.0,

	-- Return target when health falls below health_limit.
	home = { 12.9, 6.7 },

	-- Execute once after stage started, before entering cruise loop.
	entry_path = {
		{ 15.5, 7.2 },
		{ 16.3, 6.2 },
		{ 14.2, 5.9 },
		{ 14.2, 4.7 },
	},

	-- Loop forever in these waypoints after entry_path completed.
	cruise_points = {
		-- { x, y }
		{ 18.5, 4.5 },
		{ 18.9, 6.9 },
	},

	-- Navigation launch defaults.
	global_map = "training",
	launch_livox = true,
	launch_odin1 = false,
	use_sim_time = false,
}

return profile
