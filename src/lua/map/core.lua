--- @class MapPoint
--- @field name string
--- @field x number
--- @field y number

--- @alias MapTask fun(from: MapPoint, to: MapPoint): boolean

--- @class MapPathTask
--- @field run fun(): boolean 执行该边任务，返回是否成功
--- @field begin_point MapPoint 边起点
--- @field final_point MapPoint 边终点
--- @field begin_name string 边起点名称，用于日志
--- @field final_name string 边终点名称，用于日志

--- @class Map
--- @field private _points table<string, MapPoint>
--- @field private _registered table<MapPoint, boolean>
--- @field private _edges table<MapPoint, table<MapPoint, (fun(): boolean)>>
--- @field private _neighbors table<MapPoint, MapPoint[]>
local Map = {}
Map.__index = Map

--- 注册一个地图节点
--- @param name string
--- @param position { x: number, y: number }
--- @return MapPoint
function Map:point(name, position)
	if self._points[name] ~= nil then
		error("地图节点重复注册: " .. name)
	end

	local point = {
		name = name,
		x = position.x,
		y = position.y,
	}
	self._points[name] = point
	self._registered[point] = true
	return point
end

--- @param from MapPoint
--- @param to MapPoint
--- @param task (fun(): boolean)
function Map:_add_edge(from, to, task)
	if self._edges[from] == nil then
		self._edges[from] = {}
		self._neighbors[from] = {}
	end
	if self._edges[from][to] ~= nil then
		error("地图边重复注册: " .. from.name .. " -> " .. to.name)
	end

	self._edges[from][to] = task
	table.insert(self._neighbors[from], to)
end

--- 连接两个相邻节点，返回的闭包接收往返两个 Task
--- tasks[1] 为 a -> b，tasks[2] 为 b -> a
--- 注册时 core 将方向参数绑定为 (from, to)，但调用 search 返回的 Task 为零参闭包
--- @param a MapPoint
--- @param b MapPoint
--- @return fun(tasks: { [1]: MapTask, [2]: MapTask })
function Map:connect(a, b)
	if not self._registered[a] then
		error("地图边端点未注册: " .. tostring(a and a.name))
	end
	if not self._registered[b] then
		error("地图边端点未注册: " .. tostring(b and b.name))
	end

	return function(tasks)
		local forward = tasks[1]
		local backward = tasks[2]
		if forward == nil or backward == nil then
			error("地图边必须同时提供往返两个 Task: " .. a.name .. " <-> " .. b.name)
		end

		self:_add_edge(a, b, function()
			return forward(a, b)
		end)
		self:_add_edge(b, a, function()
			return backward(b, a)
		end)
	end
end

--- 搜索从 from 到 to 的路径，返回依次执行即可到达的有序 Task 表列表
--- @param from MapPoint
--- @param to MapPoint
--- @return MapPathTask[]
function Map:search(from, to)
	if not self._registered[from] then
		error("路径搜索起点未注册: " .. tostring(from and from.name))
	end
	if not self._registered[to] then
		error("路径搜索终点未注册: " .. tostring(to and to.name))
	end
	if from == to then
		return {}
	end

	local visited = { [from] = true }
	local previous = {}
	local queue = { from }
	local head = 1

	while head <= #queue do
		local node = queue[head]
		head = head + 1

		for _, next in ipairs(self._neighbors[node] or {}) do
			if not visited[next] then
				visited[next] = true
				previous[next] = node
				if next == to then
					local reversed = {}
					local cursor = to
					while cursor ~= from do
						local parent = previous[cursor]
						table.insert(reversed, {
							run = self._edges[parent][cursor],
							begin_point = parent,
							final_point = cursor,
							begin_name = parent.name,
							final_name = cursor.name,
						})
						cursor = parent
					end

					local tasks = {}
					for i = #reversed, 1, -1 do
						table.insert(tasks, reversed[i])
					end
					return tasks
				end
				table.insert(queue, next)
			end
		end
	end

	error("路径不可达: " .. from.name .. " -> " .. to.name)
end

return {
	--- @return Map
	new = function()
		return setmetatable({
			_points = {},
			_registered = {},
			_edges = {},
			_neighbors = {},
		}, Map)
	end,
}
