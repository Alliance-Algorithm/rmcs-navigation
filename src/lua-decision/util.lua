---@class UtilityModule
---@field file_exists fun(path: string): boolean
---@field find_env_setup_bash fun(): string|nil, string|nil
---@field run_command fun(command: string): boolean, string
---@field search_setup_filename fun(): string|nil, string|nil
local UtilityModule = {}

---@param path string
---@return boolean
function UtilityModule.file_exists(path)
	local file = io.open(path, "r")
	if file ~= nil then
		file:close()
		return true
	end
	return false
end

---@return string|nil, string|nil
function UtilityModule.find_env_setup_bash()
	local home = os.getenv("HOME") or ""
	local candidate_paths = {
		home ~= "" and (home .. "/env_setup.bash") or nil,
		"/root/env_setup.bash",
	}

	for _, path in ipairs(candidate_paths) do
		if path ~= nil and UtilityModule.file_exists(path) then
			return path, nil
		end
	end

	return nil, "env_setup.bash not found in $HOME or /root"
end

---@param command string
---@return boolean, string
function UtilityModule.run_command(command)
	local success, reason, code = os.execute(command)
	if success == true or success == 0 then
		return true, "ok"
	end

	local message = string.format("command failed: reason=%s, code=%s", tostring(reason), tostring(code))
	return false, message
end

---@return string|nil, string|nil
function UtilityModule.search_setup_filename()
	return UtilityModule.find_env_setup_bash()
end

return UtilityModule
