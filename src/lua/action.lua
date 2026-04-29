local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request
local blackboard = require("blackboard").singleton()

local NaN = 0 / 0

local action = {
	target = {
		x = NaN,
		y = NaN,
	},
}

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

function action:set_local_obstacle(enable)
	api.set_local_obstacle(enable)
end

function action:cancel_target()
	api.cancel_target()
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

--- 直线导航到目标点，只走当前点到目标点的方向，忽略侧向纠偏。
---
--- 原理：每次重发时将目标点沿道路垂向平移，使得 Nav2 看到的
--- 前进方向始终平行于起始方向，不会产生侧向速度分量。
---
--- @param target {x: number, y: number}
--- @param tolerance? number -- 沿道路方向的到达容差，默认 0.15
--- @param timeout? number    -- 超时秒数，默认 30
--- @param speed? number      -- 直线速度(m/s)，不传则用 Nav2 原速
function action:navigate_straight(target, tolerance, timeout, speed)
	if tolerance == nil then tolerance = 0.15 end
	if timeout == nil then timeout = 30 end

	local start = { x = blackboard.user.x, y = blackboard.user.y }

	local dx = target.x - start.x
	local dy = target.y - start.y
	local length = math.sqrt(dx * dx + dy * dy)

	if length < 0.01 then
		self:warn("navigate_straight: start and target are too close")
		self.target = { x = NaN, y = NaN }
		self:cancel_target()
		return
	end

	local ux = dx / length
	local uy = dy / length
	local nx = -uy
	local ny = ux

	local elapsed = 0
	local interval = 0.5

	while elapsed < timeout do
		local user = blackboard.user

		if speed then
			api.set_chassis_vel_override(speed * ux, speed * uy)
		end

		local perp = (user.x - start.x) * nx + (user.y - start.y) * ny
		self:navigate({
			x = target.x + perp * nx,
			y = target.y + perp * ny,
		})

		request:sleep(interval)
		elapsed = elapsed + interval

		local proj = (user.x - start.x) * ux + (user.y - start.y) * uy
		if proj >= length - tolerance then
			self.target = { x = NaN, y = NaN }
			self:cancel_target()
			if speed then
				api.set_chassis_vel_override(0, 0)
				request:sleep(0.3)
				api.clear_chassis_vel_override()
			end
			return
		end
	end

	self.target = { x = NaN, y = NaN }
	self:cancel_target()
	if speed then
		api.set_chassis_vel_override(0, 0)
		request:sleep(0.3)
		api.clear_chassis_vel_override()
	end

	self:warn(
		string.format(
			"navigate_straight: timeout (%.1fs), target (%.2f, %.2f)",
			timeout, target.x, target.y
		)
	)
end

return action
