# Batly
The Batly package is created for batly robot.
The package allows the usage of orbbec 3D camera (astra pro plus), RPLiDAR (C1) with OKDO Nano C100 in ROS Melodic distributions

# Install dependencies
## ROS
 * Please refer directly to ROS https://wiki.ros.org/melodic/Installation/Ubuntu    ( * Installation >> Desktop-Full Install )
## Arduino IDE and Install Teensy
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
 * 
