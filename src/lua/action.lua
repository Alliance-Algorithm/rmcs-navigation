local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request
local maths = require("util.math")
local clock = require("util.clock")

local NaN = 0 / 0
local kYtPerRad = 2.0
local kPtPerRad = 2.0

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

		nod_count = 0,

		yt_per_rad = kYtPerRad,
		pt_per_rad = kPtPerRad,
	},
}

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
					yt = 2 * math.pi * self.gimbal.yt_per_rad
				else
					yt = math.abs(context.y1 - context.y2) * self.gimbal.yt_per_rad
				end

				pt = math.abs(context.p1 - context.p2) * self.gimbal.pt_per_rad

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

			local nod_count = self.gimbal.nod_count
			if nod_count > 0 then
				yaw = 0 / 0

				pitch = 0 + 0.3
				api.update_gimbal_direction(yaw, pitch)
				request:sleep(0.3)

				pitch = 0 - 0.3
				api.update_gimbal_direction(yaw, pitch)
				request:sleep(0.3)

				pitch = 0
				api.update_gimbal_direction(yaw, pitch)
				self.gimbal.nod_count = nod_count - 1
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
function action:update_enable_control(enable)
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
	self.gimbal.y2 = yaw
	self.gimbal.p1 = pitch
	self.gimbal.p2 = pitch
end
--- @param times integer
function action:gimbal_nod(times)
	action:info("gimbal nod " .. times .. " times")
	self.gimbal.nod_count = self.gimbal.nod_count + times
end

--- @param speed number
function action:set_gimbal_pt(speed)
	self.gimbal.pt_per_rad = speed
end
--- @param speed number
function action:set_gimbal_yt(speed)
	self.gimbal.yt_per_rad = speed
end
function action:reset_gimbal_speed()
	self.gimbal.pt_per_rad = kPtPerRad
	self.gimbal.yt_per_rad = kYtPerRad
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

function action:toggle_record()
	return api.toggle_record()
end

--- @param position {x: number, y: number}
function action:navigate(position)
	action:info("navigate to " .. position.x .. ", " .. position.y)

	local x = position.x
	local y = position.y
	if util.check_nan(x, y) then
		return
	end

	self.target = position
end

return action
