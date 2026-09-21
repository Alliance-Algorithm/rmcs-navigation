local info = debug.getinfo(1, "S")
local script_path = info.source:sub(2)
local script_dir = script_path:match("(.*/)") or "./"
local test_util = dofile(script_dir .. "util.lua")
test_util.setup_package_path()

local assert_eq = test_util.assert_eq
local assert_false = test_util.assert_false
local assert_table_eq = test_util.assert_table_eq

local Map = require("map.core")
local scheduler = require("util.scheduler")
local clock = require("util.clock")

--- 构造一个只记录轨迹的边 Task
local function trace_task(trace, label)
	return function(from, to)
		trace[#trace + 1] = string.format("%s-%s:%s", from.name, to.name, label)
		return true
	end
end

local function assert_error(fn, message)
	local ok = pcall(fn)
	assert_false(ok, message)
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	assert_table_eq(map:search(a, a), {}, "search same point should return empty list")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	local trace = {}
	map:connect(a, b) {
		trace_task(trace, "forward"),
		trace_task(trace, "backward"),
	}

	local tasks = map:search(a, b)
	assert_eq(#tasks, 1, "direct search should return one task")
	for _, task in ipairs(tasks) do
		assert(task.begin_point == a and task.final_point == b, "task should carry begin/final points")
		assert(task.begin_name == "a" and task.final_name == "b", "task should carry point names")
		task.run()
	end
	assert_table_eq(trace, { "a-b:forward" }, "direct search should bind forward task with (a, b)")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })

	local trace = {}
	map:connect(a, b) {
		trace_task(trace, "first"),
		trace_task(trace, "backward"),
	}
	map:connect(b, c) {
		trace_task(trace, "second"),
		trace_task(trace, "backward"),
	}

	local tasks = map:search(a, c)
	assert_eq(#tasks, 1, "consecutive navigate edges should merge into one task")
	assert(tasks[1].begin_point == a and tasks[1].final_point == c, "merged task should span the whole run")
	assert(tasks[1].begin_name == "a" and tasks[1].final_name == "c", "merged task should carry run names")
	tasks[1].run()
	assert_table_eq(trace, { "a-c:first" }, "merged task should bind the first edge task with run endpoints")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })
	local d = map:point("d", { x = 3, y = 0 })
	local e = map:point("e", { x = 4, y = 0 })

	local trace = {}
	map:connect(a, b) {
		trace_task(trace, "navigate-a-b"),
		trace_task(trace, "navigate-b-a"),
	}
	map:connect_step(b, c) {
		trace_task(trace, "step-b-c"),
		trace_task(trace, "step-c-b"),
	}
	map:connect(c, d) {
		trace_task(trace, "navigate-c-d"),
		trace_task(trace, "navigate-d-c"),
	}
	map:connect(d, e) {
		trace_task(trace, "navigate-d-e"),
		trace_task(trace, "navigate-e-d"),
	}

	local tasks = map:search(a, e)
	assert_eq(#tasks, 3, "step edge should split navigate runs")
	assert(tasks[1].begin_name == "a" and tasks[1].final_name == "b", "first leg should end at step begin")
	assert(tasks[2].begin_name == "b" and tasks[2].final_name == "c", "step leg should span the step edge")
	assert(tasks[3].begin_name == "c" and tasks[3].final_name == "e", "last leg should reach the final point")
	for _, task in ipairs(tasks) do
		task.run()
	end
	assert_table_eq(
		trace,
		{ "a-b:navigate-a-b", "b-c:step-b-c", "c-e:navigate-c-d" },
		"step should stay isolated and navigate runs should merge"
	)

	for i = #trace, 1, -1 do
		trace[i] = nil
	end
	local reversed = map:search(e, a)
	assert_eq(#reversed, 3, "reverse search should keep step as barrier")
	assert(reversed[1].begin_name == "e" and reversed[1].final_name == "c", "reverse first leg should merge to step end")
	assert(reversed[2].begin_name == "c" and reversed[2].final_name == "b", "reverse step leg")
	assert(reversed[3].begin_name == "b" and reversed[3].final_name == "a", "reverse last leg")
	for _, task in ipairs(reversed) do
		task.run()
	end
	assert_table_eq(
		trace,
		{ "e-c:navigate-e-d", "c-b:step-c-b", "b-a:navigate-b-a" },
		"reverse merged legs should bind run endpoints"
	)
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })

	local trace = {}
	map:connect_step(a, b) {
		trace_task(trace, "step-a-b"),
		trace_task(trace, "step-b-a"),
	}
	map:connect(b, c) {
		trace_task(trace, "navigate-b-c"),
		trace_task(trace, "navigate-c-b"),
	}

	local tasks = map:search(a, c)
	assert_eq(#tasks, 2, "leading step edge should stay isolated")
	assert(tasks[1].begin_name == "a" and tasks[1].final_name == "b", "leading step leg")
	assert(tasks[2].begin_name == "b" and tasks[2].final_name == "c", "trailing navigate leg")
	for _, task in ipairs(tasks) do
		task.run()
	end
	assert_table_eq(trace, { "a-b:step-a-b", "b-c:navigate-b-c" }, "leading step should not merge")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })

	local noop = function(_, _)
		return true
	end
	map:connect_step(a, b) { noop, noop }
	map:connect_step(b, c) { noop, noop }

	assert_eq(#map:search(a, c), 2, "consecutive step edges should stay separate")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	local trace = {}
	map:connect(a, b) {
		trace_task(trace, "forward"),
		trace_task(trace, "backward"),
	}

	local tasks = map:search(b, a)
	assert_eq(#tasks, 1, "reverse search should return one task")
	for _, task in ipairs(tasks) do
		task.run()
	end
	assert_table_eq(trace, { "b-a:backward" }, "reverse search should bind backward task with (b, a)")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })

	local function pass(from, to)
		assert((from == a and to == b) or (from == b and to == a), "unexpected edge direction")
		return true
	end

	map:connect(a, b) { pass, pass }

	local tasks = map:search(a, b)
	assert_eq(#tasks, 1, "same function reused should produce one task")
	tasks[1].run()

	local tasks_back = map:search(b, a)
	assert_eq(#tasks_back, 1, "same function reused should produce reverse task")
	tasks_back[1].run()
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local c = map:point("c", { x = 2, y = 0 })

	local noop = function(_, _)
		return true
	end
	map:connect(a, b) { noop, noop }
	map:connect(b, c) { noop, noop }
	map:connect(a, c) { noop, noop }

	local tasks = map:search(a, c)
	assert_eq(#tasks, 1, "bfs should prefer the shortest path")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	assert_error(function()
		map:search(a, b)
	end, "unreachable search should error")
end

do
	local map = Map.new()
	map:point("a", { x = 0, y = 0 })
	assert_error(function()
		map:point("a", { x = 1, y = 1 })
	end, "duplicate point name should error")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	assert_error(function()
		map:connect(a, b) { trace_task({}, "a-b") }
	end, "connect missing backward should error")

	assert_error(function()
		map:connect(a, b) { [2] = trace_task({}, "b-a") }
	end, "connect missing forward should error")

	assert_error(function()
		map:connect_step(a, b) { trace_task({}, "a-b") }
	end, "connect_step missing backward should error")

	assert_error(function()
		map:connect_step(a, b) { [2] = trace_task({}, "b-a") }
	end, "connect_step missing forward should error")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	local noop = function(_, _)
		return true
	end
	map:connect(a, b) { noop, noop }
	assert_error(function()
		map:connect(a, b) { noop, noop }
	end, "duplicate connect should error")
end

do
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })
	local outsider = { name = "outsider", x = 9, y = 9 }

	assert_error(function()
		map:connect(a, outsider)
	end, "connect with unregistered point should error")

	map:connect(a, b)
	assert_error(function()
		map:search(a, b)
	end, "connect without tasks should register no edge")

	assert_error(function()
		map:search(outsider, a)
	end, "search with unregistered start should error")

	assert_error(function()
		map:search(a, outsider)
	end, "search with unregistered target should error")
end

do
	-- 场景式闭包：Task 签名 fun(from, to)，成功后推进 current
	clock:reset(0)
	local ctx = scheduler.new()

	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	local trace = {}
	local arrived = false

	local bb = {
		context = { current = a },
		condition = {
			near = function(target, tolerance)
				assert_eq(target, b, "navigate closure should wait on target point")
				assert_eq(tolerance, 0.1, "navigate closure tolerance")
				return arrived
			end,
		},
	}
	local action = {
		navigate = function(_, point)
			trace[#trace + 1] = "navigate:" .. point.name
		end,
	}

	local function navigate(from, to)
		action:navigate(to)
		scheduler.request:wait_until {
			monitor = function()
				return bb.condition.near(to, 0.1)
			end,
		}
		bb.context.current = to
		return true
	end

	map:connect(a, b) { navigate, navigate }

	ctx:append_task(function()
		local tasks = map:search(a, b)
		assert_eq(#tasks, 1, "scenario closure search should return one task")
		trace[#trace + 1] = "before"
		tasks[1].run()
		trace[#trace + 1] = "after"
	end)

	local function step(now)
		clock:update(now)
		ctx:spin_once()
	end

	step(0)
	assert_table_eq(trace, { "before", "navigate:b" }, "closure task should navigate then wait")
	assert_eq(bb.context.current, a, "current should not advance while waiting")

	step(1)
	assert_table_eq(trace, { "before", "navigate:b" }, "closure task should keep waiting before arrival")

	arrived = true
	step(2)
	assert_table_eq(trace, { "before", "navigate:b", "after" }, "closure task should finish after arrival")
	assert_eq(bb.context.current, b, "current should advance after closure task succeeds")
end

do
	-- 闭包 Task 抛错时 current 保持不变
	local map = Map.new()
	local a = map:point("a", { x = 0, y = 0 })
	local b = map:point("b", { x = 1, y = 0 })

	local bb = { context = { current = a } }
	local function failing(_, _)
		error("boom")
	end
	local function ok_task(from, to)
		_ = from
		bb.context.current = to
		return true
	end

	map:connect(a, b) { failing, ok_task }

	local tasks = map:search(a, b)
	local ok = pcall(tasks[1].run)
	assert_false(ok, "failing closure task should propagate error")
	assert_eq(bb.context.current, a, "current should stay when closure task fails")
end

clock:reset()

print("map.lua: ok")
