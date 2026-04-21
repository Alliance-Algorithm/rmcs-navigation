local blackboard = require("blackboard").singleton()
local request = require("util.scheduler").request
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")
local stuck_self_rescue = require("task.stuck-self-rescue")

--- @class FluctuantStuckSelfRescueOptions
--- @field max_cycles? number
--- @field settle_time? number
--- @field retreat_timeout? number
--- @field retreat_tolerance? number|{x: number, y: number}
--- @field retry_forward_timeout? number
--- @field goal_tolerance? number|{x: number, y: number}
--- @field fallback_timeout? number
--- @field fallback_max_rescue_attempts? number

local function normalize_options(options)
	options = options or {}
	assert(type(options) == "table", "options should be a table")

	local result = {
		max_cycles = options.max_cycles or 2,
		settle_time = options.settle_time or 0.25,
		retreat_timeout = options.retreat_timeout or 2.5,
		retreat_tolerance = options.retreat_tolerance or 0.2,
		retry_forward_timeout = options.retry_forward_timeout or 5.0,
		goal_tolerance = options.goal_tolerance or 0.2,
		fallback_timeout = options.fallback_timeout or 8.0,
		fallback_max_rescue_attempts = options.fallback_max_rescue_attempts or 2,
	}

	assert(type(result.max_cycles) == "number" and result.max_cycles >= 0, "max_cycles should be non-negative")
	assert(result.max_cycles % 1 == 0, "max_cycles should be an integer")
	assert(type(result.settle_time) == "number" and result.settle_time >= 0, "settle_time should be non-negative")
	assert(type(result.retreat_timeout) == "number" and result.retreat_timeout > 0, "retreat_timeout should be positive")
	assert(type(result.retry_forward_timeout) == "number" and result.retry_forward_timeout > 0, "retry_forward_timeout should be positive")
	assert(type(result.fallback_timeout) == "number" and result.fallback_timeout > 0, "fallback_timeout should be positive")
	assert(
		type(result.fallback_max_rescue_attempts) == "number" and result.fallback_max_rescue_attempts >= 0,
		"fallback_max_rescue_attempts should be non-negative"
	)
	assert(result.fallback_max_rescue_attempts % 1 == 0, "fallback_max_rescue_attempts should be an integer")

	return result
end

local function resolve_fluctuant_points(ours_zone, forward_center)
	assert(type(ours_zone) == "boolean", "ours_zone should be a boolean")
	assert(type(forward_center) == "boolean", "forward_center should be a boolean")

	local rule = blackboard.rule
	local begin, final
	if ours_zone then
		begin = rule.fluctuant_road_begin.ours
		final = rule.fluctuant_road_final.ours
	else
		begin = rule.fluctuant_road_begin.them
		final = rule.fluctuant_road_final.them
	end

	if forward_center then
		return begin, final
	end
	return final, begin
end

--- 起伏路段专用脱困：优先回退至稳定区再重试；失败后回落通用脱困。
--- @param ours_zone boolean
--- @param forward_center boolean
--- @param options? FluctuantStuckSelfRescueOptions
--- @return boolean is_success
return function(ours_zone, forward_center, options)
	action:info("开始fluctuant-stuck-self-rescue")

	local config = normalize_options(options)
	local condition = blackboard.condition

	local from, to = resolve_fluctuant_points(ours_zone, forward_center)
	if condition.near(to, config.goal_tolerance) then
		return true
	end

	for cycle = 1, config.max_cycles do
		action:warn("起伏路段疑似悬空卡住，执行回退脱困流程")

		local back_ok = navigate_to_point(from, {
			tolerance = config.retreat_tolerance,
			timeout = config.retreat_timeout,
		})
		if not back_ok then
			action:warn("起伏路段回退失败，准备回落通用脱困")
			break
		end

		if config.settle_time > 0 then
			request:sleep(config.settle_time)
		end

		local forward_ok = navigate_to_point(to, {
			tolerance = config.goal_tolerance,
			timeout = config.retry_forward_timeout,
		})
		if forward_ok then
			return true
		end

		action:warn("起伏路段重试通过失败，继续下一轮")
	end

	action:warn("起伏路段专用回退未恢复，回落通用 stuck-self-rescue")
	return stuck_self_rescue(to, {
		timeout = config.fallback_timeout,
		goal_tolerance = config.goal_tolerance,
		max_rescue_attempts = config.fallback_max_rescue_attempts,
		rescue_side_offset = 0.35,
		rescue_backtrack = 0.45,
		rescue_point_timeout = 2.0,
	})
end
