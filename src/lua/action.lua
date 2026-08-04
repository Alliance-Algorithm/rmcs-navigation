local util = require("util.math")
local api = require("api")
local request = require("util.scheduler").request
local maths = require("util.math")
local clock = require("util.clock")
local bb = require("blackboard").singleton()

local NaN = 0 / 0
local kYtPerRad = 1.5
local kPtPerRad = 2.2
local kGimbalFree = 2.2250738585072014e-308

local kMaxDwellExtend = 5 -- 静止扫描被自瞄接管时，最多补时的周期数

local kCruiseSlowScanSpeed = 3.2 -- 秒/弧度，赶路慢扫（约 20s 一圈）
local kCruiseFastScanSpeed = 1.5 -- 秒/弧度，顶点快扫（约 9.4s 一圈）

local action = {
	last_enable = false,
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

		suspend_timeline = 0,

		full_scan_start = 0,
		full_scan_stamp = 0,
	},
	climb = {
		up = false,
	},
}

--- 绑定 action 的后台任务。
--- @param scheduler Scheduler
function action:bind(scheduler)
	scheduler:append_task(function()
		local context = self.gimbal

		while true do
			local timestamp = clock:now()
			if context.mode == "free" then
				api.update_gimbal_direction(kGimbalFree, kGimbalFree)
				self.gimbal.full_scan_start = bb.user.yaw
				self.gimbal.full_scan_stamp = clock:now()
			elseif context.mode == "suspended" or timestamp < self.gimbal.suspend_timeline then
				api.update_gimbal_direction(0 / 0, 0 / 0)
				self.gimbal.full_scan_start = bb.user.yaw
				self.gimbal.full_scan_stamp = clock:now()
			else -- Need Yaw Pitch Control
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
						timestamp = timestamp - context.full_scan_stamp,
						yt = yt,
						y1 = context.y1,
						y2 = context.y2,
						pt = pt,
						p1 = context.p1,
						p2 = context.p2,
					}
					yaw = yaw + context.full_scan_start
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
			end
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

	if enable == true and self.last_enable ~= true then
		self.gimbal.full_scan_start = bb.user.yaw
		self.gimbal.full_scan_stamp = clock:now()
	end
	self.last_enable = enable
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

--- 静止停留扫描：周期内每秒轮询，期间出现过自瞄接管则补一个完整周期。
--- @param interval number 单个停留周期（秒）
function action:dwell_scan(interval)
	local extend = 0
	repeat
		local deadline = clock:now() + interval
		local interrupted = false
		while true do
			request:sleep(1)
			if bb.autoaim.should_control then
				interrupted = true
			end
			if clock:now() >= deadline then
				break
			end
		end
		if not interrupted then
			break
		end
		extend = extend + 1
		action:info("扫描期间自瞄接管，延长停留 (" .. extend .. "/" .. kMaxDwellExtend .. ")")
	until extend >= kMaxDwellExtend
end

--- 赶路慢扫：云台全圆慢扫（yt/pt 显式设置）
function action:cruise_slow_scan()
	action:set_gimbal_yt(kCruiseSlowScanSpeed)
	action:set_gimbal_pt(kPtPerRad)
	action:gimbal_scan(0, 0)
end

--- 顶点快扫：云台全圆快扫（yt/pt 显式设置）
function action:cruise_fast_scan()
	action:set_gimbal_yt(kCruiseFastScanSpeed)
	action:set_gimbal_pt(kPtPerRad)
	action:gimbal_scan(0, 0)
end

function action:gimbal_toward(yaw, pitch)
	action:info("Set gimbal to toward mode")
	self.gimbal.mode = "toward"
	self.gimbal.y1 = yaw
	self.gimbal.y2 = yaw
	self.gimbal.p1 = pitch
	self.gimbal.p2 = pitch
end

function action:gimbal_free()
	action:info("Set gimbal to free mode")
	self.gimbal.mode = "free"
end

function action:gimbal_suspend()
	action:info("Set gimbal to suspended mode")
	self.gimbal.mode = "suspended"
end

-- @param interval number
function action:gimbal_suspend_during(interval)
	self.gimbal.suspend_timeline = clock:now() + interval
end

--- @param times integer
function action:gimbal_nod(times)
	action:info("gimbal nod " .. times .. " times")
	self.gimbal.nod_count = self.gimbal.nod_count + times
end

--- @param t number
function action:set_gimbal_pt(t)
	self.gimbal.pt_per_rad = t
end

--- @param t number
function action:set_gimbal_yt(t)
	self.gimbal.yt_per_rad = t
end

function action:reset_gimbal_speed()
	self.gimbal.pt_per_rad = kPtPerRad
	self.gimbal.yt_per_rad = kYtPerRad
end

--- @param mode "normal" | "attack" | "slope"
function action:switch_motion_mode(mode)
	api.switch_motion_mode(mode)
end

--- @param yes boolean
function action:update_under_attack(yes)
	api.update_under_attack(yes)
end

--- @param enable boolean
function action:update_supercap_boost(enable)
	api.update_supercap_boost(enable)
end

--- @param enable boolean
function action:update_track_rune(enable)
	api.update_track_rune(enable)
end

--- @param enable boolean
function action:set_automatic_resurrection(enable)
	action:info("[Action] 设置自动复活为: " .. enable)
	api.set_automatic_resurrection(enable)
end

--- 触发一次重定位，红/蓝方由 referee robot_id 自动派生；
--- 服务未就绪或 robot_id 未知时本次调用被丢弃并打 WARN，不抛错。
function action:relocalize()
	action:info("[Action] 触发重定位...")
	api.relocalize()
end

--- @param config { launch_livox: boolean, launch_odin1: boolean, global_map: string, use_sim_time: boolean }
function action:restart_navigation(config)
	action:info("[Action] 正在启动导航堆栈(" .. config.global_map .. ")...")
	return api.restart_navigation(config)
end

function action:stop_navigation()
	api.stop_navigation()
end

function action:toggle_record()
	return api.toggle_record()
end

function action:start_record()
	action:info("[Action] 开始录制...")
	return api.start_record()
end

function action:abort_record()
	action:info("[Action] 中断录制...")
	return api.abort_record()
end

--- @param world_yaw number 台阶方向，world 系 XY yaw。
--- @param is_climb boolean true 上台阶，false 下台阶。
--- @return boolean success
function action:blocking_cross_step(world_yaw, is_climb)
	self.climb.up = is_climb
	api.set_climb_switch(is_climb)
	api.set_climb_direction(world_yaw)
	request:yield()

	while true do
		local status = api.get_climb_status()
		if status == 1.0 or status == -1.0 then
			api.set_climb_direction(NaN)
			return status == 1.0
		end
		request:yield()
	end
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

function action:cancel_target()
	action:info("cancel navigation target")
	self.target = { x = NaN, y = NaN }
	api.cancel_target()
end

return action
