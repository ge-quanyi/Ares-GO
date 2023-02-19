# Robomaster Computer Vision
## Get started
### Install dependecies
- OpenCV
- fmt
- Daheng libs
- eigen3
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
KERNEL==
```

install eigen3
```shell
sudo apt install libeigen3-dev
```

