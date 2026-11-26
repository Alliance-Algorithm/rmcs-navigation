local path = debug.getinfo(1, "S").source:sub(2):match("(.*/)")
local util = dofile(path .. "util.lua")
util.setup_package_path()

-- Native recording calls must never fall back to synchronous shell execution.
package.loaded["util.native"] = {
    run = function() error("record API called a synchronous shell") end,
    search_setup_resource = function() error("record API read the setup file") end,
}
local api = require("api")
local active, started, finished = false, 0, 0
api.record_active = function() return active end
api.start_record = function()
    started = started + 1
    return true, "accepted start"
end
api.finish_record = function()
    finished = finished + 1
    return false, "mapping in progress"
end
local ok, message = api.toggle_record()
util.assert_eq(ok, true, "idle toggle dispatches native start")
util.assert_eq(message, "accepted start", "native start result forwarded")
active = true
ok, message = api.toggle_record()
util.assert_eq(ok, false, "busy toggle forwards native rejection")
util.assert_eq(message, "mapping in progress", "native finish result forwarded")
util.assert_eq(started, 1, "only one launch")
util.assert_eq(finished, 1, "only one finish")
print("toggle_record.lua: ok")
