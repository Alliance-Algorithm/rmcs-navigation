local M = {}

function M.setup_package_path()
	local info = debug.getinfo(2, "S") or debug.getinfo(1, "S")
	local script_path = info.source:sub(2)
	local script_dir = script_path:match("(.*/)") or "./"
	local root = script_dir .. "../.."

	package.path = table.concat({
		root .. "/src/lua/?.lua",
		root .. "/src/lua/?/init.lua",
		root .. "/src/lua/?/?.lua",
		root .. "/test/lua/?.lua",
		package.path,
	}, ";")

	return root
end

function M.assert_eq(actual, expected, message)
	if actual ~= expected then
		error(string.format("%s: expected %s, got %s", message, tostring(expected), tostring(actual)))
	end
end

function M.assert_true(value, message)
	M.assert_eq(value, true, message)
end

function M.assert_false(value, message)
	M.assert_eq(value, false, message)
end

function M.assert_table_eq(actual, expected, message)
	M.assert_eq(#actual, #expected, message .. " length")
	for i = 1, #expected do
		M.assert_eq(actual[i], expected[i], message .. string.format("[%d]", i))
	end
end

function M.file_exists(path)
	local file = io.open(path, "r")
	if file == nil then
		return false
	end
	file:close()
	return true
end

function M.write_file(path, content)
	local file, err = io.open(path, "w")
	if file == nil then
		error(string.format("failed to open %s for writing: %s", path, tostring(err)))
	end
	file:write(content)
	file:close()
end

function M.remove_file(path)
	if M.file_exists(path) then
		os.remove(path)
	end
end

function M.with_cleanup(cleanup, action)
	local ok, result = xpcall(action, debug.traceback)
	cleanup()
	if not ok then
		error(result)
	end
	return result
end

return M
