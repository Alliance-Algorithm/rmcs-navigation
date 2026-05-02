local Region = {
	WALL = 0,
	OURS_HOME = 1,
	ROAD_REGION_BEGIN = 2,
	ROAD_REGION_1 = 3,
	ROAD_REGION_2 = 4,
	ROAD_REGION_FINAL = 5,
	OURS_HIGHLAND = 6,
}

local RegionName = {
	[Region.WALL] = "wall",
	[Region.OURS_HOME] = "ours_home",
	[Region.ROAD_REGION_BEGIN] = "road_region_begin",
	[Region.ROAD_REGION_1] = "road_region_1",
	[Region.ROAD_REGION_2] = "road_region_2",
	[Region.ROAD_REGION_FINAL] = "road_region_final",
	[Region.OURS_HIGHLAND] = "ours_highland",
}

local Map = {}
Map.__index = Map
local DEFAULT_MAP_NAME = "train_map"

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

local function is_finite_number(value)
	return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
end

local function is_positive_integer(value)
	return is_finite_number(value) and value > 0 and math.floor(value) == value
end

local function validate_data(data)
	assert(type(data) == "table", "region map data should be a table")
	assert(is_positive_integer(data.width), "region map data.width should be a positive integer")
	assert(is_positive_integer(data.height), "region map data.height should be a positive integer")
	assert(is_finite_number(data.resolution) and data.resolution > 0, "region map data.resolution should be positive")
	assert(type(data.origin) == "table", "region map data.origin should be a table")
	assert(is_finite_number(data.origin.x), "region map data.origin.x should be finite")
	assert(is_finite_number(data.origin.y), "region map data.origin.y should be finite")
	assert(type(data.rows) == "table", "region map data.rows should be a table")

	for y = 1, data.height do
		local row = data.rows[y]
		assert(type(row) == "table", "region map row " .. y .. " should be a table")
		for x = 1, data.width do
			assert(type(row[x]) == "number", "region map cell " .. y .. "," .. x .. " should be a number")
		end
	end
end

local function new_map(data)
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

local function load_map(name)
	return new_map(load_map_data(name))
end

function Map:locate(position)
	assert(type(position) == "table", "position should be a table")
	assert(type(position.x) == "number", "position.x should be a number")
	assert(type(position.y) == "number", "position.y should be a number")

	if not is_finite_number(position.x) or not is_finite_number(position.y) then
		return Region.WALL
	end

	local column = math.floor((position.x - self.origin.x) / self.resolution) + 1
	local row = self.height - math.floor((position.y - self.origin.y) / self.resolution)

	if column < 1 or column > self.width or row < 1 or row > self.height then
		return Region.WALL
	end

	local map_row = self.rows[row]
	if map_row == nil then
		return Region.WALL
	end

	return map_row[column] or Region.WALL
end

local singleton
local singleton_name

function Map.singleton(name)
	if name ~= nil then
		assert(type(name) == "string", "map name should be a string")
		if singleton == nil or singleton_name ~= name then
			singleton = load_map(name)
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

return Map
