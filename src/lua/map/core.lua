--- @class MapPoint
--- @field name string
--- @field x number
--- @field y number

--- @alias MapTask fun(from: MapPoint, to: MapPoint): boolean

--- @class MapPathTask
--- @field run fun(): boolean 执行该腿任务，返回是否成功
--- @field begin_point MapPoint 腿起点
--- @field final_point MapPoint 腿终点
--- @field begin_name string 腿起点名称，用于日志
--- @field final_name string 腿终点名称，用于日志

--- 普通边可被合并为一条腿；台阶边是合并屏障，必须单独执行
--- @alias MapEdgeKind "navigate" | "step"

--- @class MapEdge
--- @field task MapTask 方向已绑定的边任务，调用时再传入腿的起终点
--- @field kind MapEdgeKind

--- @class Map
--- @field private _points table<string, MapPoint>
--- @field private _registered table<MapPoint, boolean>
--- @field private _edges table<MapPoint, table<MapPoint, MapEdge>>
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
--- @param task MapTask
--- @param kind MapEdgeKind
function Map:_add_edge(from, to, task, kind)
	if self._edges[from] == nil then
		self._edges[from] = {}
		self._neighbors[from] = {}
	end
	if self._edges[from][to] ~= nil then
		error("地图边重复注册: " .. from.name .. " -> " .. to.name)
	end

	self._edges[from][to] = { task = task, kind = kind }
	table.insert(self._neighbors[from], to)
end

--- 注册一条边，返回的闭包接收往返两个 Task
--- tasks[1] 为 a -> b，tasks[2] 为 b -> a
--- 注册时 core 只记录方向与边的类型，search 时才把端点绑定为零参 Task
--- @param a MapPoint
--- @param b MapPoint
--- @param kind MapEdgeKind
--- @return fun(tasks: { [1]: MapTask, [2]: MapTask })
function Map:_connect(a, b, kind)
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

		self:_add_edge(a, b, forward, kind)
		self:_add_edge(b, a, backward, kind)
	end
end

--- 连接两个相邻节点，声明为普通边；搜索时连续的普通边会合并为一条腿
--- @param a MapPoint
--- @param b MapPoint
--- @return fun(tasks: { [1]: MapTask, [2]: MapTask })
function Map:connect(a, b)
	return self:_connect(a, b, "navigate")
end

--- 连接两个相邻节点，声明为台阶边；台阶边是合并屏障，必须单独执行
--- @param a MapPoint
--- @param b MapPoint
--- @return fun(tasks: { [1]: MapTask, [2]: MapTask })
function Map:connect_step(a, b)
	return self:_connect(a, b, "step")
end

--- 将有序边序列压缩为可依次执行的腿任务
--- 连续的普通边合并为一条腿，使用首条边的 Task 以整条腿的起终点调用；
--- 台阶边单独成腿，使用自身 Task 以边的起终点调用
--- @param path { from: MapPoint, to: MapPoint, edge: MapEdge }[]
--- @return MapPathTask[]
local function compress(path)
	local function make(edge, begin_point, final_point)
		return {
			run = function()
				return edge.task(begin_point, final_point)
			end,
			begin_point = begin_point,
			final_point = final_point,
			begin_name = begin_point.name,
			final_name = final_point.name,
		}
	end

	local tasks = {}
	local index = 1
	while index <= #path do
		local first = path[index]
		if first.edge.kind == "step" then
			table.insert(tasks, make(first.edge, first.from, first.to))
			index = index + 1
		else
			local last = first
			index = index + 1
			while index <= #path and path[index].edge.kind == "navigate" do
				last = path[index]
				index = index + 1
			end
			table.insert(tasks, make(first.edge, first.from, last.to))
		end
	end
	return tasks
end

--- 搜索从 from 到 to 的路径，返回依次执行即可到达的有序腿任务列表
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
							from = parent,
							to = cursor,
							edge = self._edges[parent][cursor],
						})
						cursor = parent
					end

					local path = {}
					for i = #reversed, 1, -1 do
						table.insert(path, reversed[i])
					end
					return compress(path)
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
