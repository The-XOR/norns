# norns setup

## build prerequisites

### packages
```
sudo apt-get install libevdev-dev liblo-dev libudev-dev libcairo2-dev liblua5.3-dev libavahi-compat-libdnssd-dev libasound2-dev libncurses5-dev libncursesw5-dev libsndfile1-dev libusb-1.0.0-dev libboost1.67-all-dev
```

### sources

build and install:

```
https://github.com/monome/libmonome.git
https://github.com/nanomsg/nanomsg.git
```

or use the debian repository as follows:

```
curl https://keybase.io/artfwo/pgp_keys.asc | sudo apt-key add -
echo "deb https://package.monome.org/ stretch main" | sudo tee /etc/apt/sources.list.d/norns.list
sudo apt update
sudo apt install libmonome-dev libnanomsg-dev supercollider-language supercollider-server supercollider-dev
```

## building norns

```
git clone https://github.com/The-XOR/norns.git
cd norns
git submodule update --init --recursive
./waf configure
./waf
```

this should build all the c-based components (`matron` and `ws-wrapper`.)

the `crone` audio engine consists of supercollider classes. copy files to the default location for user SC extensions

```
pushd sc
./install.sh
popd
```

## configure

- modifcare /etc/jackrd RIMUOVENDO -X seq

Rimuovere tutte le connessioni MIDI (per esempio da touchosc) verso push2
Esempio:

aconnect -d 'RtMidiOut Client' 'Ableton Push 2'
dove RtMidiOut Client e' il driver rtmidiout do touchosc bridge


## launching components

### 1. launch `crone` (audio engine)

run `crone.sh` from the norns directory. this creates a `sclang` process wrapped with `ws-wrapper`

if the crone classes are installed correctly, you should see some lines like this in output from sclang initialization:

```
-------------------------------------------------
 Crone startup

 OSC rx port: 57120
 OSC tx port: 8888
--------------------------------------------------
```

and immediately after sclang init, you should see the server being booted and some jack/alsa related messages.

### 2. launch `matron` (lua interpreter)

with the audio engine running, run `matron.sh` from the norns directory. this creates a `matron` process wrapped with `ws-wrapper`

matron waits for crone to finish loading before entering the main event loop.

### 3. launch `maiden` (web UI client)

get most [recent version](https://github.com/monome/maiden/releases)

download to `~/maiden/` and untar

execute with `./maiden.arm -debug -site ./app/build -data ~/norns/lua/`


## docs addendum

if you want to generate the docs (using ldoc) first install:

```
sudo apt-get install luarocks liblua5.1-dev
sudo luarocks install ldoc
```

to generate the docs:

`ldoc .` in the root norns folder

To read the documentation, point the browser window with Maiden loaded to [http://norns.local/doc](http://norns.local/doc) (or use IP address if this doesn't work).
