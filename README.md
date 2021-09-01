# scha_imu_board

## Overview
This package contains ROS driver nodes for Murata sensor products mainly communicate by SPI(Serial Periferal Interface).

supported devices:

 - SCHA63T Series
 - SCHA600 Series

 
 <img src="/board_hardware/IMU_board_ver2.0/scha_board(top).jpg" width=40%> <img src="/board_hardware/IMU_board_ver2.0/scha_board(back).jpg" width=40%>
 
## Building the Environment

1.
To write software to Teensy4.0, use "Arduino IDE" and "Arduino IDE + Teensyduino".
 Please install them on your Windows PC.
  
2.
  Next, place the extracted folder in an appropriate directory.

"SCHA" ⇒ scha IMU library, place it in the arudino library directory by folder.
                      This is the default location ( C:\Program Files (x86)\Arduino\libraries )
                      e.g.) C:\Program Files (x86)\Arduino\libraries\SCHA
                      
Change the valid definitions in SCHA.cpp depending on the platform you are writing to. 

By default, SCHA6xx_x03 is valid.

<br>
"scha_board" ⇒ teensy's working directory.
                      Place it anywhere you want and run scha_board.ino when compiling/writing.


3.
  After placing the folder, double-click scha_board.ino to run it in Arduino IDE.
  Click the verification button (check mark) in the upper left corner to compile the program.
  
  After the compile result is displayed at the bottom of Arduino IDE with no error, click
  Press the Write button on Teensy4.0 once.
  (Teensy 4.0 will restart automatically after writing.)
  
4.
  "Click the Terminal button in the upper right corner of the Arduino IDE to confirm that the data acquisition is successful.

## How to use

The topic name and .yaml file name are "series name"_xxx, so please change them as necessary.
There is no problem if you use them as they are.

1.
 "ros_package/scha_imu_driver" is a ros driver that runs on linux.
 Please install the ros environment on your Linux PC.

2.
Place "ros_package/scha_imu_driver" in any workspace and build it.

3.
Connect the device to your PC, and edit the device in the .yaml file in "ros_package/scha_imu_driver/config" according to the connection status.

4.
Run the appropriate launch for your device from "ros_package/scha_imu_driver/launch".



