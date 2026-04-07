local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_true = test_util.assert_true

local api = require("api")

local screen_label = "rmcs-navigation"
local restart_log = "/tmp/rmcs-navigation-restart.log"
local hardcopy_path = "/tmp/rmcs-navigation-screen-hardcopy.log"

local function screen_exists()
	local ok, _, code = os.execute(string.format("screen -S %q -Q select . >/dev/null 2>&1", screen_label))
	return ok == true or ok == 0 or code == 0
end

local function sleep_for(seconds)
	os.execute(string.format("sleep %.1f", seconds))
end

local function read_file(path)
	local file = io.open(path, "r")
	if file == nil then
		return nil
	end
	local content = file:read("*a")
	file:close()
	return content
end

local function read_screen_output()
	for _ = 1, 20 do
		os.execute(string.format("screen -S %q -p 0 -X hardcopy -h %q >/dev/null 2>&1", screen_label, hardcopy_path))
		local content = read_file(hardcopy_path)
		if content ~= nil and content ~= "" then
			return content
		end
		sleep_for(0.1)
	end
	local content = read_file(hardcopy_path)
	assert_true(content ~= nil and content ~= "", "restart_navigation should produce screen log output")
	return content
end

local function print_screen_output()
	local content = read_screen_output()
	print("restart_navigation.lua: screen output begin")
	print(content)
	print("restart_navigation.lua: screen output end")
end

local function kill_screen()
	os.execute(string.format("screen -S %q -X quit >/dev/null 2>&1", screen_label))
end

local function cleanup()
	kill_screen()
	test_util.remove_file(restart_log)
	test_util.remove_file(hardcopy_path)
end

test_util.with_cleanup(cleanup, function()
	cleanup()
	test_util.remove_file(hardcopy_path)

	local ok, message = api.restart_navigation("rmul")
	assert_true(ok, "restart_navigation should dispatch successfully")
	assert_eq(message, "ok", "restart_navigation dispatch result")

	local deadline = os.time() + 3
	while os.time() <= deadline do
		if screen_exists() then
			sleep_for(1.0)
			print_screen_output()
			kill_screen()
			assert_true(not screen_exists(), "restart_navigation should stop screen after inspection")
			print("restart_navigation.lua: ok")
			return
		end
		sleep_for(0.1)
	end

	error("restart_navigation should create rmcs-navigation screen session")
end)
