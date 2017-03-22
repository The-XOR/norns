norns.version.test_grid_timer = "0.0.1"


-- current monome device
local m = nil

grid.key = function(dev, x, y, val)
   -- use this device when we refresh
   m = dev	  
   if val == 0 then -- act on key lift
	  local seconds = y / 16;
	  local stage = x;
	  timer_start(1, seconds, -1, stage)
   end
end


local displaystage = 1;

timer = function(idx, stage)
   if(idx == 1) then
	  -- turn off the old stage
	  m:led(displaystage, 1, 0)
	  -- turn on the new stage
	  displaystage = stage
	  while displaystage > 16 do
		 displaystage = displaystage - 16
	  end
	  m:led(displaystage, 1, 1)
	  m:refresh()
   end
end