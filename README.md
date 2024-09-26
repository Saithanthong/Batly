# Batly Robot
Batly robot operates on system Ubuntu 18.04 - ROS Melodic and works together with OKDO nano c100 

# Data Information
https://drive.google.com/drive/folders/1arJcAn_-knLZPGoHLQj_kcoQgw27klcZ?usp=sharing


## ROS Installation 
https://wiki.ros.org/melodic/Installation/Ubuntu


## Arduino Installation 
https://www.arduino.cc/en/software

```
cd Downloads/
tar -xvf arduino-1.8.19-linuxaarch64.tar.xz
cd arduino 1.8.19/
sudo ./install.sh
```
## Teensyduino Installation
https://forum.pjrc.com/index.php?threads/teensyduino-1-57-beta-1.70196/
```
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
cd Downloads/
chmod 755 TeensyduinoInstall.linuxaarch64
./TeensyduinoInstall.linuxaarch64
```
### Ros-Drivers Installation
```
git clone https://github.com/ros-drivers/libuvc_ros.git
cd libuvc/
mkdir build && cd build
cmak .. && make -j4
sudo make install
sudo ldconfig
```
### Inpendencies Installation
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
# Create Workspace
```
git clone https://github.com/Saithanthong/Batly.git
cd ~/catkin_ws/
catkin build
```
# Teleop Mode
### Teleop keyboard
```
roscore
rosrun rosserial_server serial_node rtosserial_python serial_node.py /dev/ttyACM0
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### Teleop keyboard
```
roscore
rosrun rosserial_server serial_node rosserial_python serial_node.py /dev/ttyACM0
roslaunch teleop_twist_joy teleop.launch
```
# Mapping Mode
### Mapping with keyboard
```
roscore
rosrun rosserial_server serial_node rtosserial_python serial_node.py /dev/ttyACM0
roslaunch batly mapping.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### Mapping with joy-stick
```
roscore
rosrun rosserial_server serial_node rosserial_python serial_node.py /dev/ttyACM0
roslaunch batly mapping.launch
roslaunch teleop_twist_joy teleop.launch
```

# Navigation Mode
```
roscore
rosrun rosserial_server serial_node rtosserial_python /dev/ttyACM0
roslaunch batly navigation.launch
```
# 3D Camera
```

```





# Master Control Mode
### on robot
```
export ROS_MASTER  URI=http://{ MASTER_IP }:11311
export ROS_IP={ ROBOT_IP } 
```
### on master computer
```
export ROS_MASTER  URI=http://{ MASTER_IP }:11311
export ROS_IP={ MASTER_IP } 
```
