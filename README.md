# Batly Robot
The Batly package is created for Batly Robot.
The package allows the usage of orbbec 3D camera (astra pro plus), RPLiDAR (C1) with OKDO Nano C100 in ROS Melodic distributions

## ROS
 * Please refer directly to ROS https://wiki.ros.org/melodic/Installation/Ubuntu    ( * Installation >> Desktop-Full Install )
## Arduino IDE and Install Teensyduino
 * Arduino IDE 1.8.19 https://www.arduino.cc/en/software  ( * Arduino IDE 1.8.19 >> Linux ARM 64 bits )
   ```
   cd Downloads/
   tar -xvf arduino-1.8.19-linuxaarch64.tar.xz
   cd arduino-1.1.19/
   sudo ./install.sh
   ```
 * Teensyduino Download https://forum.pjrc.com/index.php?threads/teensyduino-1-57-beta-1.70196/   ( * Linux ARM64 )
   ```
   wget https://www.pjrc.com/teensy/00-teensy.rules
   sudo cp 00-teensy.rules /ect/udev/rules.d/
   sudo service udev reload
   sudo service udev restart
   cd Downloads/
   chmod 755 TeensyduinoInstall.linuxaarch64
   ./TeensyduinoInstall.linuxaarch64
   ```
 * After finished install arduino IDE and Teensyduino please refer to Arduino IDE Setup https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
### Install dependencies
   ```
   sudo apt-get install ros-melodic-slam-gmapping ros-melodic-rviz ros-melodic-robot-localization ros-melodic-tf ros-melodic-tf2*
   sudo apt-get install ros-melodic-imu-filter-madgwick ros-melodic-map-server ros-melodic-navigation
   sudo apt-get install ros-melodic-teleop-twist-keyboard ros-melodic-hector-imu-tools ros-melodic-hector-slam
   sudo apt install libgflags-dev ros-melodic-image-geometry ros-melodic-camera-info-manager
   sudo apt install ros-melodic-image-transport sudo apt install ros-melodic-image-transport libusb-1.0-0-dev libeigen3-dev
   sudo apt-get install ros-melodic-rviz-imu-plugin
   sudo apt-get install libopencv3.2
   sudo apt-get install python-catkin-tools
   sudo apt update
   sudo apt upgrade
   ```
* If unable to upgrade Follow this commands
  ```
  # move /var/lib/info/ and create new /var/lib/dpkg/info
  sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/
  sudo mkdir /var/lib/dpkg/info/
  
  # update repos and force install
  sudo apt-get update
  sudo apt-get -f install

  # move the new structure dpkg/info to old info
  sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/backup/

  # remove the new dpkg structure folder and back the old
  sudo rm -rf /var/lib/dpkg/info
  sudo mv /var/lib/dpkg/backup/ /var/lib/dpkg/info/

  # update and upgrade again
  sudo apt update
  sudo apt upgrade
  ```
## Getting start
 * Clone The Batly workspace
   ```
   git clone https://github.com/batly.git
   cd catkin_ws/
   catkin build
   ```
 * Install libuvc
   ```
   git clone https://github.com/libuvc/libuvc.git
   cd libuvc
   mkdir build && cd build
   cmake .. && make -j4
   sudo make install
   sudo ldconfig
   ```
 * Clone 3D Camera Package frome ros_astra_camera
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/orbbec/ros_astra_camera.git
    cd ~/catkin_ws
    catkin build
    ```
 * Install udev rules
    ```
    cd ~/catkin_ws
    source ./devel/setup.bash
    roscd astra_camera
    ./scripts/create_udev_rules
    sudo udevadm control --reload && sudo udevadm trigger
    source ./devel/setup.bash 
    ```
 * Clone LiDAR Package frome rplidar_ros
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/Slamtec/rplidar_ros.git
    cd ~/catkin_ws
    catkin build
    ```
 * rplidar ros package setup
    ```
    # Check the authority of rplidar's serial-port 
    ls -l /dev |grep ttyUSB

    # Add the authority of write: (such as /dev/ttyUSB0)
    sudo chmod 666 /dev/ttyUSB0
    ```








  
