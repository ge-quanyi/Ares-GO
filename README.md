# RoboMaster Computer Vision

Ares2023 Season

## Get started
### Install dependecies
- OpenCV
- fmt
- Daheng libs
- eigen3
- matplot-cpp
- Flask

- zmq

### Settings
Give serial port permission
```shell
sudo usermod -a -G dialout user_name
```
Bind stm32 serial port
```shell
lsusb   # remember device id
...
sudo  vim  /etc/udev/rules.d/stm.rules
```
add the contents to this file
```vim
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="stm"
```

install eigen3
```shell
sudo apt install libeigen3-dev
```
install zmq
```shell
sudo apt install libzmq3-dev
pip install pyzmq
git clone https://github.com/zeromq/libzmq.git
# then build the source
```

install flask

```shell
pip install flask[global]
```

## Usage

```shell
cd <work_dir>
git clone <remote_url>
cd Ares_GO
mkdir build && cd build
cmake ..
make 
```

### Run Autoaim 

```shell
./Ares_CV
```

### Run web server

```shell
cd <work_dir>/Ares-GO
python/python3 ./scripts/app.py
```

