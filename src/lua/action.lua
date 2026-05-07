local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request

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

-- 注意exchange_17mm_bullet switch_mode confirm_revive 不能同时进行，保护策略（详见component.cc）会清空同时发出的其他命令
function action:exchange_17mm_bullet(amount)
	amount = blackboard.game.exchanged_bullet + amount
	api.exchange_17mm_bullet(amount)
end

function action:switch_mode(mode)
	api.switch_mode(mode)
end

-- 配合can_confirm_free_revive使用
function action:confirm_revive()
	api.confirm_revive()
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
