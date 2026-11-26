local path = debug.getinfo(1, "S").source:sub(2):match("(.*/)")
local util = dofile(path .. "util.lua")
util.setup_package_path()
local eq = util.assert_eq
local fake = { ready = false, active = false, status = "idle", starts = 0, finishes = 0, aborts = 0, nods = 0 }
local api = setmetatable({
    record_ready = function() return fake.ready end,
    record_active = function() return fake.active end,
    record_status = function() return fake.status end,
    start_record = function()
        fake.starts = fake.starts + 1
        fake.active, fake.status = true, "starting"
        return true, "ok"
    end,
    finish_record = function()
        fake.finishes = fake.finishes + 1
        return true, "ok"
    end,
    abort_record = function() fake.aborts = fake.aborts + 1; return true, "ok" end,
    update_enable_control = function(value) fake.enabled = value end,
    switch_topic_forward = function(value) fake.forward = value end,
    update_gimbal_direction = function(_, pitch)
        if pitch == 0.3 then fake.nods = fake.nods + 1 end
    end,
    switch_motion_mode = function() error("record changed chassis mode") end,
    restart_navigation = function() error("record restarted navigation") end,
}, { __index = function() return function() end end })
package.loaded.api = api
local Record = require("mode.record")
local bb = require("blackboard").singleton()
local action = require("action")
local Scheduler = require("util.scheduler")
local now = 0
local record = Record.new()
local function tick(knob, dt)
    now = now + (dt or 0.05)
    bb.meta.timestamp, bb.play.rotary_knob = now, knob or bb.play.rotary_knob
    return record:update()
end
local function triple()
    for _ = 1, 3 do tick("MIDDLE"); tick("DOWN") end
end
triple()
eq(fake.starts, 0, "wait for background initial probe")
fake.ready = true
tick("MIDDLE")
-- Held DOWN is a single edge, and UP clears unfinished gestures.
tick("DOWN"); tick("DOWN"); tick("DOWN")
eq(fake.starts, 0, "held down does not retrigger")
tick("UP")
triple()
eq(fake.starts, 1, "triple starts recording")
eq(record.state, "starting", "acceptance is not recording acknowledgement")
eq(fake.nods, 0, "no start signal before recorder acknowledgement")
for _ = 1, 60 do tick(nil, 0.01) end
eq(record.state, "starting", "slow background launch leaves control loop responsive")
fake.status = "recording test-bag"
for _ = 1, 25 do tick(nil, 0.01) end
eq(record.state, "recording", "acknowledged recording stage")
triple()
eq(fake.finishes, 1, "triple requests finish")
fake.status = "mapping test-bag"
triple()
eq(fake.starts, 1, "mapping ignores start gesture")
eq(fake.finishes, 1, "mapping ignores finish gesture")
fake.status = "success test.pcd"
for _ = 1, 250 do tick(nil, 0.01) end
eq(record.state, "idle", "release after final status and queued nods")
eq(fake.nods, 4, "two start nods and two queued end nods")
eq(fake.enabled, false, "control disabled after completion")
eq(fake.forward, false, "forwarding disabled after completion")

-- Startup failure releases the scheduler without fabricating a start signal.
fake.active = false
triple()
fake.status = "failed: slow launch failed"
for _ = 1, 30 do tick(nil, 0.01) end
eq(record.state, "idle", "background launch failure releases ownership")
eq(fake.nods, 4, "failed launch has no start signal")

-- Service restart adopts a pending launch, and still signals its real acknowledgement.
fake.active, fake.status = true, "starting"
record = Record.new()
tick("MIDDLE")
eq(record.state, "starting", "adopt starting workflow as starting")
fake.status = "recording survived-bag"
for _ = 1, 160 do tick(nil, 0.01) end
eq(record.state, "recording", "adopted launch reaches recording")
eq(fake.nods, 6, "adopted pending launch gets two nods")
record:release()

-- Cancellation dispatches abort without a shell in the scheduler's close path.
local scheduler = Scheduler.new()
local task = scheduler:append_task(require("task.auto_record"))
scheduler:spin_once()
task.cancel()
scheduler:spin_once()
eq(fake.aborts, 1, "cancel recording requests native abort")
eq(action.gimbal.nod_count, 0, "cancel clears nods")
print("record.lua: ok")
