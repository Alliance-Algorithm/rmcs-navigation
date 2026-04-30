local intent = {}

function intent.new()
	local details = {
		a = 1,
		b = 2,
	}
	return setmetatable(details, intent)
end

return intent
