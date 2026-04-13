local option = {}

function option:list_all()
	local names = {}
	for name, value in pairs(self) do
		if string.find(name, "^qos_overrides") then
			goto continue
		end
		if type(value) ~= "function" then
			names[#names + 1] = string.format("  - %s: %s", name, tostring(value))
		end
		::continue::
	end

	return table.concat(names, "\n")
end

return option
