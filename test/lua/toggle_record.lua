local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_true = test_util.assert_true

local original_util_native = package.loaded["util.native"]
local original_api = package.loaded["api"]
local original_action = package.loaded["action"]

local function restore_modules()
	package.loaded["util.native"] = original_util_native
	package.loaded["api"] = original_api
	package.loaded["action"] = original_action
end

test_util.with_cleanup(restore_modules, function()
	do
		local calls = {}
		package.loaded["util.native"] = {
			search_setup_resource = function()
				return "/tmp/env_setup.bash", nil
			end,
			run = function(command)
				calls[#calls + 1] = command
				if command == "tmux has-session -t map-build 2>/dev/null" then
					return false, "missing"
				end
				return true, "ok"
			end,
		}
		package.loaded["api"] = nil

		local api = require("api")
		local ok, message = api.start_record()
		assert_true(ok, "start_record should dispatch successfully")
		assert_eq(message, "ok", "start_record dispatch result")
		assert_eq(#calls, 2, "start_record should check map build then dispatch once")
		assert_eq(calls[1], "tmux has-session -t map-build 2>/dev/null", "start_record should check map build state first")

		local command = calls[2]
		assert_true(command:match("tmux kill%-session %-t record%-livox" ) ~= nil, "start_record should kill existing record session")
		assert_true(command:match("tmux new%-session %-d %-s record%-livox" ) ~= nil, "start_record should create record session")
		assert_true(command:match("if %[% %-d /home/ubuntu %]" ) ~= nil, "start_record should prefer /home/ubuntu")
		assert_true(command:match("elif %[% %-d /root %]" ) ~= nil, "start_record should fallback to /root")
		assert_true(command:match("lidar%|imu") ~= nil, "start_record should filter lidar/imu topics only")
		assert_true(command:match("undistort") == nil, "start_record should not record undistort topics")
		assert_true(command:match("record_root=\\%$record_parent/record%-livox" ) ~= nil, "start_record should use record-livox directory")
		assert_true(command:match("ros2 bag record %-o" ) ~= nil, "start_record should invoke ros2 bag record")
		assert_true(command:find("rm -f /tmp/rmcs-navigation/last-record-bag", 1, true) ~= nil, "start_record should clear stale bag state")
		assert_true(command:find("/tmp/rmcs-navigation/last-record-bag", 1, true) ~= nil, "start_record should record bag state")
	end

	do
		local calls = {}
		package.loaded["util.native"] = {
			search_setup_resource = function()
				return "/tmp/env_setup.bash", nil
			end,
			run = function(command)
				calls[#calls + 1] = command
				return true, "ok"
			end,
		}
		package.loaded["api"] = nil

		local api = require("api")
		local ok, message = api.start_record()
		assert_true(not ok, "start_record should refuse while mapping")
		assert_eq(message, "map build in progress", "start_record refusal reason")
		assert_eq(#calls, 1, "start_record should stop after map build check")
	end

	do
		local calls = {}
		package.loaded["util.native"] = {
			search_setup_resource = function()
				return "/tmp/env_setup.bash", nil
			end,
			run = function(command)
				calls[#calls + 1] = command
				return true, "ok"
			end,
		}
		package.loaded["api"] = nil

		local api = require("api")
		local ok, message = api.finish_record()
		assert_true(ok, "finish_record should dispatch successfully")
		assert_eq(message, "ok", "finish_record dispatch result")
		assert_eq(#calls, 1, "finish_record should invoke util.run once")

		local command = calls[1]
		assert_true(command:find("record-map.sh", 1, true) ~= nil, "finish_record should start mapping script")
		assert_true(command:find("tmux new-session -d -s map-build", 1, true) ~= nil, "finish_record should create map build session")
		assert_true(command:find("/tmp/record-map-dispatch.log", 1, true) ~= nil, "finish_record should log dispatch output")
	end

	do
		local calls = {}
		package.loaded["util.native"] = {
			search_setup_resource = function()
				return "/tmp/env_setup.bash", nil
			end,
			run = function(command)
				calls[#calls + 1] = command
				if command == "tmux has-session -t record-livox 2>/dev/null" then
					return true, "ok"
				end
				return true, "ok"
			end,
		}
		package.loaded["api"] = nil

		local api = require("api")
		local ok, message = api.toggle_record()
		assert_true(ok, "toggle_record should stop existing recorder successfully")
		assert_eq(message, "ok", "toggle_record stop result")
		assert_eq(#calls, 2, "toggle_record stop path should invoke util.run twice")
		assert_eq(calls[1], "tmux has-session -t record-livox 2>/dev/null", "toggle_record should check session existence first")
		assert_true(calls[2]:match("tmux send%-keys %-t record%-livox:bag C%-c" ) ~= nil, "toggle_record should send Ctrl+C to active record session")
	end

	do
		local calls = {}
		package.loaded["util.native"] = {
			search_setup_resource = function()
				return "/tmp/env_setup.bash", nil
			end,
			run = function(command)
				calls[#calls + 1] = command
				if command == "tmux has-session -t record-livox 2>/dev/null" then
					return false, "missing"
				end
				if command == "tmux has-session -t map-build 2>/dev/null" then
					return false, "missing"
				end
				return true, "ok"
			end,
		}
		package.loaded["api"] = nil
		package.loaded["action"] = nil

		local action = require("action")
		local ok, message = action:toggle_record()
		assert_true(ok, "action toggle_record should start recorder successfully")
		assert_eq(message, "ok", "action toggle_record start result")
		assert_eq(calls[1], "tmux has-session -t record-livox 2>/dev/null", "action toggle_record should query tmux session")
		assert_eq(calls[2], "tmux has-session -t map-build 2>/dev/null", "action toggle_record should check map build state")
		assert_true(calls[3]:match("tmux new%-session %-d %-s record%-livox" ) ~= nil, "action toggle_record should start record session")
	end

	print("toggle_record.lua: ok")
end)
