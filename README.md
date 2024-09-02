# Batly Robot
Batly robot operates on system Ubuntu 18.04 - ROS Melodic and works together with OKDO nano c100 

## installation ROS


## Installation Arduino
https://www.arduino.cc/en/software

```
cd Downloads/
tar -xvf arduino-1.8.19-linuxaarch64.tar.xz
cd arduino 1.8.19/
sudo ./install.sh
```

## Installation Teensyduino
```
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
cd Downloads/
chmod 755 TeensyduinoInstall.linuxaarch64
./TeensyduinoInstall.linuxaarch64
```


```
git clone https://github.com/ros-drivers/libuvc_ros.git
cd libuvc/
mkdir build && cd build
cmak .. && make -j4
sudo make install
sudo ldconfig
```


#installation Inpendencies
```
sudo apt-get install ros-melodic-slam-gmapping 
sudo apt-get install ros-melodic-rviz
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-tf
sudo apt-get install ros-melodic-tf2*
sudo apt-get install ros-melodic-imu-filter-madgwick
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-teleo-twist-keyboard
sudo apt-get install ros-melodic-hector-imu_tools
sudo apt-get install ros-melodic-hector-slam
sudo apt-get install ros-melodic-rviz-imu-plugin
sudo apt-get install python-catkin-tools
sudo apt install linbgflags-dev
sudo apt install ros-melodic-image-geometry
sudo apt install ros-melodic-camera-info-manager
sudo apt install ros-melodic-image-trasport 
sudo apt install libusb-1.0-0-dev
sudo apt install libeigen3-dev
sudo apt update
sudo apt upgrade
```

Create Workspace
```
mkdir catkin_ws
cd catkin_ws/
git clone https://github.com/Saithanthong/Batly.git
