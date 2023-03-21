# Robomaster Computer Vision
## Get started
### Install dependecies
- OpenCV
- fmt
- Daheng libs
- eigen3
- matplot-cpp
### Set 
Giving rights to serial
```shell
sudo usermod -a -G dialout user_name
```
lock serial id to 'stm'
```shell
lsusb   # remember device id
...
sudo  vim  /etc/udev/rules.d/10-local.rules
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
```
