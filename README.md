# AutoC2X-OC

![movie](https://user-images.githubusercontent.com/23014935/76400182-39570c00-63c3-11ea-81cb-a6b84179406d.gif)

AutoC2X is cooperative awareness driving software, extension for Autoware and OpenC2X. AutoC2X-OC is fork project from OpenC2X.

## description

[Autoware](https://gitlab.com/autowarefoundation/autoware.ai) is open-source autonomous driving software. [OpenC2X](https://www.ccs-labs.org/software/openc2x/) is open-source cooperative ITS software, able to communicate with other vehicles following ITS-G5. AutoC2X is an extension for Autoware and OpenC2X. Using this software, you can get other vehicle information run by Autoware.

## AutoC2X-AW
AutoC2X-OC collaborate with [AutoC2X-AW](https://github.com/esakilab/AutoC2X-AW) at laptop. So you should install and run AutoC2X-AW in laptop.

## Set up the environment
Open a terminal and run the command:

    $ sudo apt-get install libzmq3-dev libboost-all-dev protobuf-compiler libprotobuf-dev libgps-dev gpsd gpsd-clients libnl-3-200 libnl-3-dev libnl-genl-3-200 libnl-genl-3-dev sqlite3 libsqlite3-dev tmux asn1c build-essential cmake doxygen

    $ echo "deb https://dl.bintray.com/fynnh/debian xenial main" | sudo tee -a /etc/apt/sources.list

    $ sudo apt-get update

    $ sudo apt-get install openc2x



## Generate source from .asn before compiling
In the terminal, navigate to asn1 folder in common:

    $ cd /path_to_openc2x/OpenC2X/common/asn1/
    $ ./generate.sh



## Compiling
In the terminal, navigate to your project folder and configure the project:

    $ cd path_to_openc2x/OpenC2X/
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make all



## Configurations
- The configuration of the modules can be changed in 'OpenC2X/<module>/config/'
- Make sure to change the interface name and the stationId in 'common/config/config.xml'
  as per your setup.
- For setting up OpenC2X kernel, go through 'kernel-patches/README.txt'

### hardware Setup
In AutoC2X, each vehicle is supposed to equip one laptop and one router. Autoware and AutoC2X-AW is running in laptop. AutoC2X-OC is running in router.

![hardware](https://user-images.githubusercontent.com/23014935/76481753-b59a2f80-6455-11ea-9134-4b5376bf75c4.png)

### network setup
You should allocate IPv4 address to laptops and routers. Below image is example.

![network](https://user-images.githubusercontent.com/23014935/76482009-50930980-6456-11ea-9155-3abf0788592b.png)


## Start AutoC2X
You should start AutoC2X below order.

1. Start AutoC2X-AW at receiver laptop
2. Start AutoC2X-OC at receiver router <- # sh ./runOpenC2X.sh r <laptop's ip address>
3. Start AutoC2X-OC at sender router <- # sh ./runOpenC2X.sh s <laptop's ip address>
4. Start AutoC2X-AW at sender laptop

If you have followed the above steps, then you are ready to run OpenC2X.
In the terminal, navigate to scripts:

    $ cd path_to_openc2x/OpenC2X/scripts/
    # sh ./runOpenC2X.sh [r|s] <laptop's ip address>

For the GUI, open 'webSite/index.html' in your browser. You can also trigger denms from there.
You can stop your experiment by running stopOpenC2X.sh in another terminal.

## setup the environment
1. clone the repogitory
```
$ git clone https://github.com/nlohmann/json.git
$ git clone https://github.com/ppianpak/rosbridgecpp.git
```
