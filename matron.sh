#!/bin/sh
cd /home/patch/norns/
echo "Starting matron"
/home/patch/norns/build/ws-wrapper/ws-wrapper ws://*:5555 /home/patch/norns/build/matron/matron &
echo "matron.sh: job done"
