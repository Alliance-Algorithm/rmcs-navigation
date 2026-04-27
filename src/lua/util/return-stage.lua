local ReturnStage = {
	before_fluctuant = "before_fluctuant",
	on_fluctuant = "on_fluctuant",
	after_fluctuant = "after_fluctuant",
}

function ReturnStage.resolve_escape_route(stage)
	if stage == ReturnStage.before_fluctuant then
		return "direct"
	end
	if stage == ReturnStage.on_fluctuant then
		return "fluctuant_road"
	end
	if stage == ReturnStage.after_fluctuant then
		return "onestep"
	end

	error("unknown return stage: " .. tostring(stage))
end

return ReturnStage
