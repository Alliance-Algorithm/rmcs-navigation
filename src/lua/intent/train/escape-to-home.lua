local blackboard = require("blackboard").singleton()
local action = require("action")
local fsm = require("util.fsm")
local navigate_to_point = require("task.navigate-to-point")

local EscapeToHomeIntent = {}
EscapeToHomeIntent.__index = EscapeToHomeIntent

local State = {
	follow_route = "follow_route",
	done = "done",
	failed = "failed",
}

local M = {
	State = State,
}

local function select_point(point, ours_zone)
	if type(point.x) == "number" and type(point.y) == "number" then
		return point
	end
	return ours_zone and point.ours or point.them
end

function M.new(args)
	args = args or {}
	assert(type(args) == "table", "args should be a table")
	if args.ours_zone == nil then
		args.ours_zone = true
	end
	assert(type(args.ours_zone) == "boolean", "args.ours_zone should be a boolean")

	return setmetatable({
		ours_zone = args.ours_zone,
		route = args.route or "ours_home",
		tolerance = args.tolerance or 0.15,
		timeout = args.timeout or 10.0,
		phase = "none",
		status = "running",
		machine = nil,
	}, EscapeToHomeIntent)
end

function EscapeToHomeIntent:phase_name()
	return self.phase
end

function EscapeToHomeIntent:create_route_targets()
	local rule = blackboard.rule
	local road_begin = select_point(rule.road_zone_begin, self.ours_zone)
	local road_final = select_point(rule.road_zone_final, self.ours_zone)
	local way_point_1 = select_point(rule.road_zone_way_point_1, self.ours_zone)
	local way_point_2 = select_point(rule.road_zone_way_point_2, self.ours_zone)
	local resupply_zone = select_point(rule.resupply_zone, self.ours_zone)

	if self.route == "highland" then
		return {
			{ name = "road_zone_final", point = road_final },
			{ name = "road_zone_way_point_2", point = way_point_2 },
			{ name = "road_zone_way_point_1", point = way_point_1 },
			{ name = "road_zone_begin", point = road_begin },
			{ name = "resupply_zone", point = resupply_zone },
		}
	end

	if self.route == "road_region_final" then
		return {
			{ name = "road_zone_way_point_2", point = way_point_2 },
			{ name = "road_zone_way_point_1", point = way_point_1 },
			{ name = "road_zone_begin", point = road_begin },
			{ name = "resupply_zone", point = resupply_zone },
		}
	end

	if self.route == "road_region_2" then
		return {
			{ name = "road_zone_way_point_1", point = way_point_1 },
			{ name = "road_zone_begin", point = road_begin },
			{ name = "resupply_zone", point = resupply_zone },
		}
	end

	if self.route == "road_region_1" then
		return {
			{ name = "road_zone_begin", point = road_begin },
			{ name = "resupply_zone", point = resupply_zone },
		}
	end

	if self.route == "ours_home" or self.route == "road_region_begin" or self.route == "direct" then
		return {
			{ name = "resupply_zone", point = resupply_zone },
		}
	end

	error("unknown train escape route: " .. tostring(self.route))
end

function EscapeToHomeIntent:create_machine(ctx)
	local machine = fsm:new(State.follow_route)

	machine:use({
		state = State.follow_route,
		enter = function()
			self.phase = State.follow_route
			action:update_chassis_mode("SPIN")
			ctx.run_job("train_escape_follow_route", function()
				action:info("train/escape-to-home: 回家路径 -> " .. self.route)
				for _, target in ipairs(self:create_route_targets()) do
					action:info(string.format(
						"train/escape-to-home: 导航到%s (x=%.2f, y=%.2f)",
						target.name,
						target.point.x,
						target.point.y
					))
					local ok = navigate_to_point(target.point, {
						tolerance = self.tolerance,
						timeout = self.timeout,
					})
					if not ok then
						action:warn("train/escape-to-home: 导航到" .. target.name .. "失败")
						return false
					end
				end
				return true
			end)
		end,
		event = function(handle)
			local job = ctx.job_state()
			if not job.done then
				return
			end

			if job.success then
				handle:set_next(State.done)
			else
				handle:set_next(State.failed)
			end
		end,
	})

	machine:use({
		state = State.done,
		enter = function()
			self.phase = State.done
			self.status = "success"
			action:info("train/escape-to-home: 已抵达补给点")
		end,
		event = function() end,
	})

	machine:use({
		state = State.failed,
		enter = function()
			self.phase = State.failed
			self.status = "failed"
		end,
		event = function() end,
	})

	assert(machine:init_ready(State), "train escape-to-home intent fsm init_ready failed")
	return machine
end

function EscapeToHomeIntent:spin(ctx)
	assert(type(ctx) == "table", "ctx should be a table")
	if self.machine == nil then
		self.machine = self:create_machine(ctx)
	end
	if self.status == "running" then
		self.machine:spin_once()
	end
	return self.status
end

return M
