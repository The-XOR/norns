local m = {}

m.key = function(n,z)
  if n==2 and z==1 then
    _menu.set_page("HOME")
  elseif n==3 and z==1 then
    m.sleep = true
    if _menu.m.TAPE.rec.sel == 3 then
      audio.tape_record_stop()
    end
    _menu.redraw()
    norns.shutdown()
  end
end

m.enc = norns.none

m.redraw = function()
  screen.clear()
  screen.move(48,40)
  if m.sleep then
    screen.level(1)
    if norns.is_shield then
      screen.move(10,40)
      screen.text("when the green light")
      screen.move(10,48)
      screen.text("stops blinking")
      screen.move(10,56)
      screen.text("disconnect power")
      screen.move(10,64)
      screen.text("and va a ciapà i ratt.")

    else
      screen.text("sleep.")
    end
  else
    screen.level(15)
    screen.text("turn off?")
  end
  --TODO do an animation here! fade the volume down
  screen.update()
end

m.init = norns.none
m.deinit = norns.none

return m
