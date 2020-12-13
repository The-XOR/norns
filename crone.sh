#!/bin/sh
cd /home/patch/norns/
# do not try reserving device (disable dbus)
export JACK_NO_AUDIO_RESERVATION=1

# start jack clients
# scsynth -u 57122 -i 2 -o 2 &
/home/patch/norns/build/crone/crone &
/home/patch/norns/build/ws-wrapper/ws-wrapper ws://*:5556 /usr/bin/sclang &
