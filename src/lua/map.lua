local Region = {
	WALL = 0,
	OURS_HOME = 1,
	THEM_HOME = 2,
	OURS_FLUCTUANT = 3,
	THEM_FLUCTUANT = 4,
	OURS_TRAPEZOIDAL_HIGHLAND = 5,
	THEM_TRAPEZOIDAL_HIGHLAND = 6,
	OURS_ROAD_TO_FLUCTUANT = 7,
	THEM_ROAD_TO_FLUCTUANT = 8,
	OURS_ROAD_TO_HIGHLAND = 9,
	THEM_ROAD_TO_HIGHLAND = 10,
	OURS_HIGHLAND = 11,
	THEM_HIGHLAND = 12,
}

local RegionName = {
	[Region.WALL] = "wall",
	[Region.OURS_HOME] = "ours_home",
	[Region.THEM_HOME] = "them_home",
	[Region.OURS_FLUCTUANT] = "ours_fluctuant",
	[Region.THEM_FLUCTUANT] = "them_fluctuant",
	[Region.OURS_TRAPEZOIDAL_HIGHLAND] = "ours_trapezoidal_highland",
	[Region.THEM_TRAPEZOIDAL_HIGHLAND] = "them_trapezoidal_highland",
	[Region.OURS_ROAD_TO_FLUCTUANT] = "ours_road_to_fluctuant",
	[Region.THEM_ROAD_TO_FLUCTUANT] = "them_road_to_fluctuant",
	[Region.OURS_ROAD_TO_HIGHLAND] = "ours_road_to_highland",
	[Region.THEM_ROAD_TO_HIGHLAND] = "them_road_to_highland",
	[Region.OURS_HIGHLAND] = "ours_highland",
	[Region.THEM_HIGHLAND] = "them_highland",
}

local Map = {}
Map.__index = Map
local DEFAULT_MAP_NAME = "rmuc"

local function dirname(path)
	return path:match("^(.*)/[^/]*$") or "."
end

local function source_dir()
	local source = debug.getinfo(1, "S").source
	if source:sub(1, 1) == "@" then
		return dirname(source:sub(2))
	end
	return "."
end

local function load_map_data(name)
	local lua_dir = source_dir()
	local candidates = {
		lua_dir .. "/../maps/" .. name .. ".lua",
		lua_dir .. "/../../maps/" .. name .. ".lua",
	}
	local errors = {}

	for _, path in ipairs(candidates) do
		local chunk, err = loadfile(path)
		if chunk then
			return chunk()
		end
		errors[#errors + 1] = path .. ": " .. tostring(err)
	end

	error("failed to load region map " .. name .. ":\n" .. table.concat(errors, "\n"))
end

local function validate_data(data)
	assert(type(data) == "table", "region map data should be a table")
	assert(type(data.width) == "number", "region map data.width should be a number")
	assert(type(data.height) == "number", "region map data.height should be a number")
	assert(type(data.resolution) == "number", "region map data.resolution should be a number")
	assert(type(data.origin) == "table", "region map data.origin should be a table")
	assert(type(data.origin.x) == "number", "region map data.origin.x should be a number")
	assert(type(data.origin.y) == "number", "region map data.origin.y should be a number")
	assert(type(data.rows) == "table", "region map data.rows should be a table")
	assert(#data.rows == data.height, "region map row count should equal data.height")

	for y, row in ipairs(data.rows) do
		assert(type(row) == "table", "region map row should be a table")
		assert(#row == data.width, "region map row " .. y .. " width should equal data.width")
	end
end

function Map.new(data)
	validate_data(data)

	return setmetatable({
		width = data.width,
		height = data.height,
		resolution = data.resolution,
		origin = data.origin,
		names = data.names or RegionName,
		rows = data.rows,
	}, Map)
end

function Map.from_file(name)
	return Map.new(load_map_data(name))
end

function Map:locate(position, y)
	local x = position
	if type(position) == "table" then
		x = position.x
		y = position.y
	end

	local column = math.floor((x - self.origin.x) / self.resolution) + 1
	local row = self.height - math.floor((y - self.origin.y) / self.resolution)

	if column < 1 or column > self.width or row < 1 or row > self.height then
		return Region.WALL
	end

	return self.rows[row][column]
end

local singleton
local singleton_name

function Map.singleton(name)
	if name ~= nil then
		assert(type(name) == "string", "map name should be a string")
		if singleton == nil or singleton_name ~= name then
			singleton = Map.from_file(name)
			singleton_name = name
		end
		return singleton
	end

	if singleton == nil then
		return Map.singleton(DEFAULT_MAP_NAME)
	end

	return singleton
end

function Map.current_name()
	return singleton_name or DEFAULT_MAP_NAME
end

Map.Region = Region
Map.names = RegionName

return Map
