local blackboard = require("blackboard").singleton()
local clock = require("util.clock")
local request = require("util.scheduler").request
local util = require("util.math")
local action = require("action")
local navigate_to_point = require("task.navigate-to-point")

local function distance(a, b)
	local dx = a.x - b.x
	local dy = a.y - b.y
	return math.sqrt(dx * dx + dy * dy)
end

local function vector_length(x, y)
	return math.sqrt(x * x + y * y)
end

--- @class StuckSelfRescueOptions
--- @field timeout? number
--- @field monitor_interval? number
--- @field stall_window? number
--- @field min_progress? number
--- @field goal_tolerance? number|{x: number, y: number}
--- @field stuck_ignore_tolerance? number
--- @field max_rescue_attempts? number
--- @field rescue_side_offset? number
--- @field rescue_backtrack? number
--- @field rescue_point_timeout? number
--- @field rescue_point_tolerance? number|{x: number, y: number}

local function normalize_options(options)
	options = options or {}
	assert(type(options) == "table", "options should be a table")

	local result = {
		timeout = options.timeout or 12.0,
		monitor_interval = options.monitor_interval or 0.1,
		stall_window = options.stall_window or 1.2,
		min_progress = options.min_progress or 0.05,
		goal_tolerance = options.goal_tolerance or 0.15,
		stuck_ignore_tolerance = options.stuck_ignore_tolerance or 0.35,
		max_rescue_attempts = options.max_rescue_attempts or 2,
		rescue_side_offset = options.rescue_side_offset or 0.6,
		rescue_backtrack = options.rescue_backtrack or 0.4,
		rescue_point_timeout = options.rescue_point_timeout or 2.5,
		rescue_point_tolerance = options.rescue_point_tolerance or 0.15,
	}

	assert(type(result.timeout) == "number" and result.timeout > 0, "timeout should be positive")
	assert(
		type(result.monitor_interval) == "number" and result.monitor_interval > 0,
		"monitor_interval should be positive"
	)
	assert(
		type(result.stall_window) == "number" and result.stall_window > 0,
		"stall_window should be positive"
	)
	assert(
		type(result.min_progress) == "number" and result.min_progress >= 0,
		"min_progress should be non-negative"
	)
	assert(
		type(result.stuck_ignore_tolerance) == "number" and result.stuck_ignore_tolerance >= 0,
		"stuck_ignore_tolerance should be non-negative"
	)
	assert(
		type(result.max_rescue_attempts) == "number" and result.max_rescue_attempts >= 0,
		"max_rescue_attempts should be non-negative"
	)
	assert(
		type(result.max_rescue_attempts) == "number" and result.max_rescue_attempts % 1 == 0,
		"max_rescue_attempts should be an integer"
	)
	assert(
		type(result.rescue_side_offset) == "number" and result.rescue_side_offset >= 0,
		"rescue_side_offset should be non-negative"
	)
	assert(
		type(result.rescue_backtrack) == "number" and result.rescue_backtrack >= 0,
		"rescue_backtrack should be non-negative"
	)
	assert(
		type(result.rescue_point_timeout) == "number" and result.rescue_point_timeout > 0,
		"rescue_point_timeout should be positive"
	)

	return result
end

local function build_rescue_targets(goal, side_offset, backtrack)
	local current = {
		x = blackboard.user.x,
		y = blackboard.user.y,
	}

	local dx = goal.x - current.x
	local dy = goal.y - current.y
	local norm = vector_length(dx, dy)
	if norm < 1e-6 then
		dx = math.cos(blackboard.user.yaw)
		dy = math.sin(blackboard.user.yaw)
		norm = vector_length(dx, dy)
	end

	if norm < 1e-6 then
		dx = 1.0
		dy = 0.0
		norm = 1.0
	end

	local ux = dx / norm
	local uy = dy / norm
	local sx = -uy
	local sy = ux

	return {
		{
			x = current.x + sx * side_offset,
			y = current.y + sy * side_offset,
		},
		{
			x = current.x - sx * side_offset,
			y = current.y - sy * side_offset,
		},
		{
			x = current.x - ux * backtrack,
			y = current.y - uy * backtrack,
		},
	}
end

--- @param goal {x: number, y: number}
--- @param config StuckSelfRescueOptions
--- @param deadline number
--- @return "success" | "stuck" | "timeout"
local function monitor_progress(goal, config, deadline)
	local condition = blackboard.condition
	local previous = {
		x = blackboard.user.x,
		y = blackboard.user.y,
	}
	local stall_begin = nil

	while true do
		if condition.near(goal, config.goal_tolerance) then
			return "success"
		end

		local now = clock:now()
		if now >= deadline then
			return "timeout"
		end

		local current = {
			x = blackboard.user.x,
			y = blackboard.user.y,
		}
		local moved = distance(current, previous)
		previous = current

		if moved >= config.min_progress then
			stall_begin = nil
		else
			stall_begin = stall_begin or now
			if now - stall_begin >= config.stall_window then
				if not condition.near(goal, config.stuck_ignore_tolerance) then
					return "stuck"
				end
			end
		end

		request:sleep(config.monitor_interval)
	end
end

--- @param goal {x: number, y: number}
--- @param config StuckSelfRescueOptions
--- @param deadline number
--- @return boolean rescued
local function execute_rescue(goal, config, deadline)
	local targets = build_rescue_targets(goal, config.rescue_side_offset, config.rescue_backtrack)
	for _, target in ipairs(targets) do
		local now = clock:now()
		local remaining = deadline - now
		if remaining <= 0 then
			return false
		end

		local timeout = math.min(config.rescue_point_timeout, remaining)
		local ok = navigate_to_point(target, {
			tolerance = config.rescue_point_tolerance,
			timeout = timeout,
		})
		if ok then
			return true
		end
	end

	return false
end

--- 通用卡死自救：检测无进展并执行侧向/回退子目标脱困。
--- @param goal {x: number, y: number}
--- @param options? StuckSelfRescueOptions
--- @return boolean is_success
return function(goal, options)
	assert(type(goal) == "table", "goal should be a table")
	assert(type(goal.x) == "number", "goal.x should be a number")
	assert(type(goal.y) == "number", "goal.y should be a number")
	assert(not util.check_nan(goal.x, goal.y), "goal should not be NaN")
	action:info("开始stuck-self-rescue")

	local config = normalize_options(options)
	local condition = blackboard.condition
	local deadline = clock:now() + config.timeout

	if condition.near(goal, config.goal_tolerance) then
		return true
	end

	local rescue_count = 0
	while true do
		action:navigate(goal)
		local status = monitor_progress(goal, config, deadline)
		if status == "success" then
			return true
		end
		if status == "timeout" then
			action:warn("stuck-self-rescue 超时，未能抵达目标")
			return false
		end

		rescue_count = rescue_count + 1
		if rescue_count > config.max_rescue_attempts then
			action:warn("stuck-self-rescue 超过最大脱困次数")
			return false
		end

		action:warn("检测到无进展，开始执行脱困子目标")
		local rescued = execute_rescue(goal, config, deadline)
		if not rescued then
			action:warn("脱困子目标执行失败")
			return false
		end
	end
end
