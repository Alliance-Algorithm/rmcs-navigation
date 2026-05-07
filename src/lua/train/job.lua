local Job = {}
Job.__index = Job

function Job.new(args)
	return setmetatable({
		scheduler = args.scheduler,
		action = args.action,
		label = args.label,
		handle = nil,
		name = nil,
		done = false,
		success = false,
	}, Job)
end

function Job:reset_status()
	self.done = false
	self.success = false
end

function Job:cancel()
	if self.handle ~= nil then
		self.handle.cancel()
		self.handle = nil
	end
	self.name = nil
	self:reset_status()
end

function Job:run(name, fn)
	self:cancel()
	self.name = name
	self:reset_status()

	self.handle = self.scheduler:append_task(function()
		local ok, result = xpcall(fn, debug.traceback)
		self.handle = nil
		self.name = nil
		self.done = true

		if not ok then
			self.success = false
			self.action:fuck(string.format("%s job '%s' failed:\n%s", self.label, name, result))
			return
		end

		self.success = (result ~= false)
		if not self.success then
			self.action:warn(string.format("%s job '%s' finished with false", self.label, name))
		end
	end)
end

return Job
