#!/bin/sh

apt-get install libzmq3-dev libboost-all-dev protobuf-compiler libprotobuf-dev libgps-dev gpsd gpsd-clients libnl-3-200 libnl-3-dev libnl-genl-3-200 libnl-genl-3-dev sqlite3 libsqlite3-dev tmux asn1c build-essential cmake doxygen libnl-route-3-dev
echo "deb [trusted=yes] https://dl.bintray.com/fynnh/debian xenial main" | sudo tee -a /etc/apt/sources.list
apt-get update
apt-get install openc2x
cd $HOME/AutoC2X-OC
mkdir $HOME/AutoC2X-OC/sources
git clone https://github.com/ppianpak/rosbridgecpp.git
git clone https://github.com/nlohmann/json.git
mkdir build
cd $HOME/AutoC2X-OC/build
cmake ..
make all
