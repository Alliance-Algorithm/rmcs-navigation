local M = {}

--- Return true if any argument is NaN.
--- @param ... any
--- @return boolean
function M.check_nan(...)
	for i = 1, select("#", ...) do
		local value = select(i, ...)
		if type(value) == "number" and value ~= value then
			return true
		end
	end
	return false
end

--- @class ScanningConfig
---
--- @field timestamp number 时间戳，秒
---
--- @field yt number Yaw 轴扫描周期，秒
--- @field y1 number Yaw 轴扫描起始，rad
--- @field y2 number Yaw 轴扫描终点，rad
---
--- @field pt number Pitch 扫描周期，秒
--- @field p1 number Pitch 扫描起始，rad
--- @field p2 number Pitch 扫描终点，rad
---
--- @see Pitch 扫描区域为 [-0.35, +0.35]，Yaw 为一周，[0, 2*pi]
---      右手系，X 向前，Y 向左，X 正方向 Yaw 为 0

--- @param config ScanningConfig
--- @return number, number target Yaw 和 Pitch 的目标角度
function M.scanning_signal(config)
	local tau = 2 * math.pi

	local wave = function(timestamp, period, v1, v2)
		local phase = (timestamp % period) / period
		local alpha = 0.5 - 0.5 * math.cos(2 * math.pi * phase)

		return v1 + (v2 - v1) * alpha
	end

	local timestamp = config.timestamp
	local yaw = 0
	if config.y1 == 0 and config.y2 == 0 then
		local phase = (timestamp % config.yt) / config.yt
		yaw = 2 * phase * tau
	else
		local span = (config.y2 - config.y1) % tau
		yaw = (config.y1 + wave(timestamp, config.yt, 0, span)) % tau
	end

	local pitch = wave(timestamp, config.pt, config.p1, config.p2)

	return yaw, pitch
end

return M
