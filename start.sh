#!/bin/sh
cd /home/patch/norns/
aconnect -d 'RtMidiOut Client' 'Ableton Push 2'
/home/patch/norns/crone.sh  &
/home/patch/norns/matron.sh &
/home/patch/maiden/start.sh &
