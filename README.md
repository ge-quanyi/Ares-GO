# Robomaster Computer Vision
## Get started
### Install dependecies
- OpenCV
- fmt
- Daheng libs
- pybind11
- flask
- craw
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

