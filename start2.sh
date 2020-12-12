sudo systemctl stop orac
sudo systemctl stop mec
cd /home/patch/norns/
./stop.sh
./crone.sh  &
./matron.sh &
/maiden/start.sh &
