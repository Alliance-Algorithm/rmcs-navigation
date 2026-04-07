local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_true = test_util.assert_true
local assert_false = test_util.assert_false

package.loaded["util.clock"] = nil
local clock = require("util.clock")

clock:reset()
assert_false(clock:is_ready(), "clock should not be ready after reset without timestamp")
assert_eq(clock:now(), 0, "clock reset default timestamp")

clock:update(1.25)
assert_true(clock:is_ready(), "clock should be ready after update")
assert_eq(clock:now(), 1.25, "clock update timestamp")

clock:reset(3.5)
assert_true(clock:is_ready(), "clock reset with timestamp should keep clock ready")
assert_eq(clock:now(), 3.5, "clock reset timestamp")

clock:reset()
assert_false(clock:is_ready(), "clock reset should clear ready state")
assert_eq(clock:now(), 0, "clock reset should clear timestamp")

print("clock.lua: ok")
