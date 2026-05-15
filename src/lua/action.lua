local util = require("util.math")

local clock = require("util.clock")

local api = require("api")
local request = require("util.scheduler").request

local blackboard = require("blackboard").singleton()

local DEFAULT_RELOCALIZE_TIMEOUT_SEC = 30.0

local RelocalizeState = { IDLE = 0, IN_FLIGHT = 1, SUCCEEDED = 2, FAILED = 3 }

local NaN = 0 / 0

local action = {
	target = {
		x = NaN,
		y = NaN,
	},
}


local function pose_unavailable(x, y, yaw)
	return x == nil or y == nil or yaw == nil or util.check_nan(x, y, yaw)
end


--- 绑定 action 的后台任务。
--- @param scheduler Scheduler
function action:bind(scheduler)
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

function action:update_gimbal_direction(angle)
	api.update_gimbal_direction(angle)
end

function action:update_gimbal_dominator(name)
	api.update_gimbal_dominator(name)
end

function action:switch_controller(mode)
	api.switch_controller(mode)
end

function action:update_chassis_mode(mode)
	api.update_chassis_mode(mode)
end

function action:update_enable_autoaim(enable)
	api.update_enable_autoaim(enable)
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
	return send_and_await(self, "wide", api.relocalize_wide, NaN, NaN, NaN, timeout_sec)
end

function action:relocalize_status()
	return api.relocalize_status()
end

function action:exchange_17mm_bullet(amount)
	amount = blackboard.game.exchanged_bullet + amount
	api.exchange_17mm_bullet(amount)
end

local last_switch_sent = -5

function action:switch_mode(mode)
	local now = clock:now()
	if now - last_switch_sent < 5.0 then
		return
	end
	last_switch_sent = now
	api.switch_mode(mode)
end

function action:confirm_revive()
	api.confirm_revive()
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
