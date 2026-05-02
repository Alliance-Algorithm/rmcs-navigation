local action = require("action")

local BlackboardLogger = {}

function BlackboardLogger.attach(scheduler, blackboard)
	local request = require("util.scheduler").request

	scheduler:append_task(function()
		while true do
			request:sleep(1.0)

			local u = blackboard.user
			local g = blackboard.game
			local p = blackboard.play
			local m = blackboard.meta
			local r = blackboard.referee
			local rhp = r.robots_hp

			action:info(string.format(
				"BB| user: hp=%d bullet=%d pwr_limit=%d pwr=%.0f buf=%.0f out=%s pos=(%.2f,%.2f) yaw=%.1f mode=%s cool=%d heat_lim=%d 42mm=%d 17mm=%d spd=%d shoot_ts=%.1f",
				u.health,
				u.bullet,
				u.chassis_power_limit,
				u.chassis_power,
				u.chassis_buffer_energy,
				tostring(u.chassis_output_status),
				u.x, u.y,
				u.yaw,
				u.mode,
				u.shooter_cooling,
				u.shooter_heat_limit,
				u.bullet_42mm,
				u.fortress_17mm_bullet,
				u.initial_speed,
				u.shoot_timestamp
			))

			action:info(string.format(
				"BB| game: stage=%s outpost=%d base=%d hero=%d eng=%d inf1=%d inf2=%d time=%d gold=%d ammo=%d dart=%s fort=%s bigE=%s smlE=%s pos_h=(%.1f,%.1f) pos_e=(%.1f,%.1f) pos_i1=(%.1f,%.1f) pos_i2=(%.1f,%.1f)",
				g.stage,
				g.outpost_health,
				g.base_health,
				g.hero_health,
				g.engineer_health,
				g.infantry_1_health,
				g.infantry_2_health,
				g.remaining_time,
				g.gold_coin,
				g.exchangeable_ammunition_quantity,
				tostring(g.our_dart_nmber_of_hits),
				tostring(g.fortress_occupied),
				tostring(g.big_energy_mechanism_activated),
				tostring(g.small_energy_mechanism_activated),
				g.hero_position.x, g.hero_position.y,
				g.engineer_position.x, g.engineer_position.y,
				g.infantry_1_position.x, g.infantry_1_position.y,
				g.infantry_2_position.x, g.infantry_2_position.y
			))

			action:info(string.format(
				"BB| referee: sync_ts=%d rid=%d rhp=(1=%d 2=%d 3=%d 4=%d 7=%d res=%d outpost=%d base=%d) free_rv=%s inst_rv=%s rv_cost=%d ex_bullet=%d ex_count=%d s_mode=%d e_activ=%s red=%d blue=%d",
				r.sync_timestamp,
				r.robot_id,
				rhp.ally_1, rhp.ally_2, rhp.ally_3, rhp.ally_4,
				rhp.ally_7, rhp.reserved,
				rhp.outpost, rhp.base,
				tostring(r.can_confirm_free_revive),
				tostring(r.can_exchange_instant_revive),
				r.instant_revive_cost,
				r.exchanged_bullet,
				r.remote_bullet_exchange_count,
				r.sentry_mode,
				tostring(r.energy_mechanism_activatable),
				r.red_score,
				r.blue_score
			))

			action:info(string.format(
				"BB| play: rs=%s ls=%s | meta: ts=%.1f fsm=%s ret=%s",
				p.rswitch,
				p.lswitch,
				m.timestamp,
				m.fsm_state,
				m.fsm_return_stage
			))
		end
	end)
end

return BlackboardLogger
