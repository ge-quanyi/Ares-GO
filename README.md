# Ares-Sentry-CV

2023赛季哨兵代码

```txt
├─camera
│  ├─include
│  └─src
├─detector
│  ├─include
│  ├─model
│  └─src
├─include
├─params
├─predictor
│  ├─include
│  └─src
├─scripts
│  └─templates
├─serial
└─wit-motion
```

## Get started
### Denpendencies
- OpenCV
- fmt
- Daheng libs
- eigen3
- matplot-cpp
- Flask

- zmq

### Installation

**OpenVino2022**

detail contents see: <https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html#doxid-openvino-docs-install-guides-installing-openvino-from-archive-linux>

> Note: This only install runtime, not include develop tools.

```shell
# create install dir
sudo mkdir /opt/intel
# download
cd <user_home>/Downloads   

curl -L https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.3/linux/l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64.tgz --output openvino_2022.3.0.tgz   

tar -xf openvino_2022.3.0.tgz   

sudo mv l_openvino_toolkit_ubuntu20_2022.3.0.9052.9752fafe8eb_x86_64 /opt/intel/openvino_2022.3.0   

#install
cd /opt/intel/openvino_2022.3.0/   

sudo -E ./install_dependencies/install_openvino_dependencies.sh

cd /opt/intel 
sudo ln -s openvino_2022.3.0 openvino2022
# source enviroment
source /opt/intel/openvino2022/setupvars.sh

```

**Eigen**

```shell
sudo apt install libeigen3-dev
```
**zmq**
```shell
sudo apt install libzmq3-dev
pip install pyzmq
git clone https://github.com/zeromq/libzmq.git
# then build the source
```

**flask**

```shell
pip install flask[global]
```

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

## Usage

```shell
cd <work_dir>
git clone <remote_url>
cd Ares_GO
mkdir glog && mkdir video
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



