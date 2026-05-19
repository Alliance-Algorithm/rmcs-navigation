local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request
local maths = require("util.math")
local clock = require("util.clock")
local blackboard = require("blackboard").singleton()

local NaN = 0 / 0
local kYtPerRad = 1.2
local kPtPerRad = 1.5
local DEFAULT_RELOCALIZE_TIMEOUT_SEC = 30.0

local RelocalizeState = { IDLE = 0, IN_FLIGHT = 1, SUCCEEDED = 2, FAILED = 3 }

local action = {
	target = {
		x = NaN,
		y = NaN,
	},
	gimbal = {
		mode = "",
		y1 = 0.,
		y2 = 0,
		p1 = 0 + 0.3,
		p2 = 0 - 0.3,
	},
}

local function pose_unavailable(x, y, yaw)
	return x == nil or y == nil or yaw == nil or util.check_nan(x, y, yaw)
end

--- 绑定 action 的后台任务。
--- @param scheduler Scheduler
function action:bind(scheduler)
	scheduler:append_task(function()
		local context = self.gimbal

		while true do
			local timestamp = clock:now()
			local yaw, pitch
			if context.mode == "scanning" then
				local yt, pt
				if context.y1 == 0 and context.y2 == 0 then
					yt = 2 * math.pi * kYtPerRad
				else
					yt = math.abs(context.y1 - context.y2) * kYtPerRad
				end

				pt = math.abs(context.p1 - context.p2) * kPtPerRad

				yaw, pitch = maths.scanning_signal {
					timestamp = timestamp,
					yt = yt,
					y1 = context.y1,
					y2 = context.y2,
					pt = pt,
					p1 = context.p1,
					p2 = context.p2,
				}
			elseif context.mode == "toward" then
				yaw = context.y1
				pitch = context.p1
			end
			api.update_gimbal_direction(yaw, pitch)

			request:yield()
		end
	end)

	--- 定期重发当前导航目标，避免导航链路在规划失败或短暂中断后停滞；
	scheduler:append_task(function()
		while true do
			local x = self.target.x
			local y = self.target.y
			if not util.check_nan(x, y) then
				api.send_target(x, y)
			end
			request:sleep(2.0)
		end
	end)

	--- 在目标切换时立即发送一次，减少等待下一个周期重发的延迟。
	scheduler:append_task(function()
		local last = { x = NaN, y = NaN }
		while true do
			local x = self.target.x
			local y = self.target.y

			if x ~= last.x or y ~= last.y then
				if not util.check_nan(x, y) then
					api.send_target(x, y)
				end
			end

			last = { x = x, y = y }
			request:yield()
		end
	end)
end

--- @param enable boolean
function action:switch_navigation(enable)
	api.update_enable_control(enable)
end

function action:switch_topic_forward(enable)
	api.switch_topic_forward(enable)
end

function action:info(message)
	api.info(message)
end

function action:warn(message)
	api.warn(message)
end

function action:fuck(message)
	api.fuck(message)
end

function action:gimbal_scan(y1, y2)
	action:info("Set gimbal to scanning mode")
	self.gimbal.mode = "scanning"
	self.gimbal.y1 = y1
	self.gimbal.y2 = y2
	self.gimbal.p1 = 0 + 0.2
	self.gimbal.p2 = 0 - 0.2
end
function action:gimbal_toward(yaw, pitch)
	action:info("Set gimbal to toward mode")
	self.gimbal.mode = "toward"
	self.gimbal.y1 = yaw
	self.gimbal.y1 = yaw
	self.gimbal.p1 = pitch
	self.gimbal.p2 = pitch
end

function action:update_gimbal_dominator(name)
	api.update_gimbal_dominator(name)
end

--- @param mode "normal" | "attack" | "road" | "step" | "slope"
function action:switch_motion_mode(mode)
	api.switch_motion_mode(mode)
end

--- @param yes boolean
function action:update_under_attack(yes)
	api.update_under_attack(yes)
end

function action:restart_navigation(config)
	return api.restart_navigation(config)
end

function action:stop_navigation()
	api.stop_navigation()
end

--- @return ok boolean
--- @return st table
local function send_and_await(self, mode, fn, x, y, yaw, timeout_sec)
	if not fn(x, y, yaw) then
		local st = api.relocalize_status()
		self:warn(string.format("reloc skip %s | state=%d msg=%s", mode, st.state, tostring(st.message)))
		return false, st
	end

	local deadline = clock:now() + (timeout_sec or DEFAULT_RELOCALIZE_TIMEOUT_SEC)
	while true do
		local st = api.relocalize_status()
		if st.state == RelocalizeState.SUCCEEDED then
			self:info(string.format("reloc ok [%s] score=%.4f conf=%.3f", mode, st.fitness_score, st.confidence))
			return true, st
		end
		if st.state ~= RelocalizeState.IN_FLIGHT then
			self:warn(string.format("reloc fail [%s] score=%.4f conf=%.3f | %s", mode, st.fitness_score, st.confidence,
				tostring(st.message)))
			return false, st
		end
		if clock:now() > deadline then
			st.state = RelocalizeState.FAILED
			st.success = false
			st.message = "lua wait timeout"
			self:warn(string.format("reloc fail [%s] | %s", mode, st.message))
			return false, st
		end
		request:sleep(0.1)
	end
end

function action:relocalize_initial(x, y, yaw, timeout_sec)
	return send_and_await(self, "initial", api.relocalize_initial, x, y, yaw, timeout_sec)
end

function action:relocalize_local(timeout_sec)
	local user = blackboard.user
	if pose_unavailable(user.x, user.y, user.yaw) then
		self:warn("reloc skip local (LIO/TF lost, no validator anchor)")
		return false, nil
	end

	return send_and_await(self, "local", api.relocalize_local, user.x, user.y, user.yaw, timeout_sec)
end

function action:relocalize_wide(timeout_sec)
	local user = blackboard.user
	local x, y, yaw = user.x, user.y, user.yaw
	if pose_unavailable(x, y, yaw) then
		x, y, yaw = 0.0, 0.0, 0.0
	end
	return send_and_await(self, "wide", api.relocalize_wide, x, y, yaw, timeout_sec)
end

function action:relocalize_status()
	return api.relocalize_status()
end

--- @param position {x: number, y: number}
function action:navigate(position)
	local x = position.x
	local y = position.y
	if util.check_nan(x, y) then
		return
	end

	self.target = position
end

return action
