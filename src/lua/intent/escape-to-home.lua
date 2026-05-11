local Intent = function()
	local fsm

	-- 1 从 enemy to road
	-- 1 执行完就跳到 2
	-- spin task

	-- 2
	-- 2 执行完就跳到 3

	-- 3
	-- 3 执行完就跳到 1

	-- Check State
	if true then
		local area = map:localize { x = 0, y = 0 }
		fsm:strat_with("1")
	end

	while true do
		-- spin fsm
	end
end

return Intent
