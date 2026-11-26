local api = require("api")
local action = require("action")
local clock = require("util.clock")
local Scheduler = require("util.scheduler")
local bb = require("blackboard").singleton()

local Record = {}
Record.__index = Record

function Record.new()
	return setmetatable({
		previous = "UNKNOWN",
		edges = {},
		state = "idle",
		poll_at = 0,
		alive_at = 0,
		signals = 0,
		signalling = false,
	}, Record)
end

-- Count only observed MIDDLE -> DOWN transitions, within one rolling 2 s window.
function Record:gesture(value, now)
	if value == "UNKNOWN" or value == "UP" or (self.last_time and now < self.last_time) then
		self.edges = {}
	end
	self.last_time = now
	while #self.edges > 0 and now - self.edges[1] > 2 do
		table.remove(self.edges, 1)
	end
	local triggered = false
	if self.previous == "MIDDLE" and value == "DOWN" then
		self.edges[#self.edges + 1] = now
		if #self.edges == 3 then
			self.edges = {}
			triggered = true
		end
	end
	self.previous = value
	return triggered
end

function Record:acquire()
	action:update_enable_control(false)
	action:cancel_target()
	action:switch_topic_forward(false)
	action:gimbal_suspend()
	self.scheduler = Scheduler.new()
	action:bind(self.scheduler)
	bb.context.input_epoch = bb.context.input_epoch + 1
end

function Record:release()
	action:update_enable_control(false)
	action:switch_topic_forward(false)
	action:gimbal_suspend()
	action.gimbal.nod_count = 0
	self.signals = 0
	self.signalling = false
	self.scheduler = nil
	self.state = "idle"
	self.edges = {}
	bb.context.input_epoch = bb.context.input_epoch + 1
end

function Record:update()
	local now = bb.meta.timestamp
	clock:update(now)
	if not self.initialized then
		-- The initial process probe runs on the native worker, never in this tick.
		if not api.record_ready() then
			return false
		end
		self.initialized = true
		if api.record_active() then
			local stage = api.record_status():match("^(%S+)")
			self:acquire()
			self.state = (stage == "starting" or stage == "recording") and stage or "mapping"
		end
	end
	local triggered = self:gesture(bb.play.rotary_knob, now)
	if triggered then
		if self.state == "idle" then
			local ok, message = api.start_record()
			if ok then
				self:acquire()
				self.state = "starting"
				self.poll_at = now
				action:info("旋钮三连：正在启动录制")
			else
				action:warn("启动录制失败：" .. tostring(message))
			end
		elseif self.state == "starting" or self.state == "recording" then
			local ok, message = api.finish_record()
			if ok then
				self.state = "mapping"
				self.signals = self.signals + 1
				action:info("旋钮三连：结束录制，开始建图")
			else
				action:warn("结束录制失败：" .. tostring(message))
			end
		end
	end

	if self.state ~= "idle" and self.state ~= "finished" and now >= self.poll_at then
		self.poll_at = now + 0.2
		local status = api.record_status()
		local stage = status:match("^(%S+)")
		if stage == "recording" and self.state == "starting" then
			self.state = "recording"
			self.signals = self.signals + 1
			action:info("雷达录制已启动")
		elseif stage == "success" or stage == "aborted" or stage == "failed:" then
			if stage == "failed:" then
				action:warn(status)
			else
				action:info(status)
			end
			self.state = "finished"
		elseif now >= self.alive_at then
			self.alive_at = now + 2
			if not api.record_active() then
				action:warn("录制/建图进程已退出：" .. status)
				self.state = "finished"
			end
		end
	end

	if self.scheduler then
		-- Queue signals so a second nod request cannot overwrite an in-flight nod.
		if self.signalling and action.gimbal.nod_count == 0 then
			self.signalling = false
			action:gimbal_suspend()
		end
		if not self.signalling and self.signals > 0 then
			self.signals = self.signals - 1
			self.signalling = true
			action:gimbal_toward(0 / 0, 0)
			action.gimbal.suspend_timeline = 0
			action:gimbal_nod(2)
		end
		action:update_enable_control(self.signalling)
		self.scheduler:spin_once()
		if self.state == "finished" and not self.signalling and self.signals == 0 then
			self:release()
		end
	end
	return self.state ~= "idle"
end

-- The public task adapter is task.auto_record; no endpoint callbacks are replaced.
function Record.run()
	local record = Record.new()
	local cleanup <close> = setmetatable({}, {
		__close = function()
			if record.state ~= "idle" then
				if record.state == "starting" or record.state == "recording" then
					api.abort_record()
				end
				record:release()
			end
		end,
	})
	while true do
		if record:update() then
			Scheduler.request:exclusive()
		else
			Scheduler.request:yield()
		end
	end
end

return Record
