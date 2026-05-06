local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request
local blackboard = require("blackboard").singleton()

local NaN = 0 / 0

local RelocalizeState = {
	IDLE = 0,
	IN_FLIGHT = 1,
	SUCCEEDED = 2,
	FAILED = 3,
}

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

function action:update_chassis_mode(mode)
	api.update_chassis_mode(mode)
end

function action:update_chassis_vel(x, y)
	api.update_chassis_vel(x, y)
end

--- 三模式共享：发起 → 轮询 → 终态。
--- @return ok    boolean   是否被服务端 accept
--- @return st    table?    末态 status（被 in-flight 拦截时为 nil）
local function send_and_await(self, mode, fn, x, y, yaw)
	if not fn(x, y, yaw) then
		self:warn(string.format("reloc skip %s (in_flight)", mode))
		return false, nil
	end
	while true do
		local st = api.relocalize_status()
		if st.state == RelocalizeState.SUCCEEDED then
			self:info(string.format(
				"reloc ok [%s] score=%.4f conf=%.3f", mode, st.fitness_score, st.confidence))
			return true, st
		end
		if st.state ~= RelocalizeState.IN_FLIGHT then
			self:warn(string.format(
				"reloc fail [%s] score=%.4f conf=%.3f | %s",
				mode, st.fitness_score, st.confidence, tostring(st.message)))
			return false, st
		end
		request:sleep(0.1)
	end
end

--- INITIAL 模式：x/y/yaw 是 GICP 起点（建图原点，一般为（0,0,0））。开局重定位用。
function action:relocalize_initial(x, y, yaw)
	return send_and_await(self, "initial", api.relocalize_initial, x, y, yaw)
end

--- LOCAL 模式（SC-driven）：持续重试直到成功。
--- blackboard.user.{x,y,yaw} 仅作 validator 锚点（拦镜像错配，红蓝对称场地有双高峰现象）；
--- LIO/TF 丢失（nil/NaN）时直接返回失败，不重试也不切 WIDE。
function action:relocalize_local()
	local attempt = 1
	while true do
		local user = blackboard.user
		if pose_unavailable(user.x, user.y, user.yaw) then
			self:warn("reloc skip local (LIO/TF lost, no validator anchor)")
			return false, nil
		end

		local ok, st = send_and_await(self, "local_", api.relocalize_local, user.x, user.y, user.yaw)
		if ok then return true, st end
		self:warn(string.format("local failed, retrying (attempt %d)", attempt))
		attempt = attempt + 1
		request:yield()
	end
end

--- WIDE 模式：blackboard.user 作 validator prior（NaN 时退化到原点 + 无 prior 验收，pointlio崩了就和原点配）。
function action:relocalize_wide()
	local user = blackboard.user
	local x, y, yaw = user.x, user.y, user.yaw
	if pose_unavailable(x, y, yaw) then x, y, yaw = 0.0, 0.0, 0.0 end
	return send_and_await(self, "wide", api.relocalize_wide, x, y, yaw)
end

function action:relocalize_status()
	return api.relocalize_status()
end

function action:restart_navigation(config)
	return api.restart_navigation(config)
end

function action:stop_navigation()
	api.stop_navigation()
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
