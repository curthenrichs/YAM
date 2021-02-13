# YAM
YAM (Your Automated Machine) is a mobile robot that I have been building on and off for the past several years. I use it as a test/experimentation platform and as a learning experience.

## Architecture
The compute architecture for YAM consists of two Arduino microcontrollers and a nVidia Jetson Nano. One microcontroller is repsonsible for low-level drivetrain and spatial sensing. The other inferfaces with YAMs frontpanel IO. 

A schematic and software stack diagram are provided in the documentation (comming soon).

## ROS Package Structure

### YAM Core


To connect to Xbox Driver follow this:
https://askubuntu.com/questions/724246/can-i-run-blueman-from-command-line
https://www.thegeekpub.com/16265/using-xbox-one-controllers-on-a-raspberry-pi/#:~:text=Pairing%20Bluetooth%20Xbox%20One%20Controllers%20to%20the%20Raspberry%20Pi&text=With%20ERTM%20enabled%20the%20Xbox,adapter%20or%20a%20USB%20cable.

### YAM Firmware



### YAM Messages
YAM provides a set of custom messages, services, and actions used to easily transfer data within the stack when standard ROS messages are insufficient.

Messages
- CartesianDrive ~ Exposes cartesian drivetrain control from the microcontroller to ROS
- DifferentialDrive ~ Exposes differential drivetrain control from the microcontroller to ROS.
- RobotState ~ Presents the current low-level navigation firmware state.
- Ultrasonics ~ Presents the current readings of the three ultrasonic sensors attached to YAM. This is used instead of multiple Range messages being sent from the firmware layer. 
- Infrareds ~ Presents the current readings of the two infrared sensors attached to YAM. This is used instead of multiple Range messages being sent from the firmware layer.      

Services
- None right now :(

Actions
- None right now :(

### YAM Description
This is where any descriptive files such as URDFs or custom meshes are stored. Currently I have written a simple URDF so that I can visualize the robot in RVIZ.

### Documentation
All relevant development or explanatory documenation is provided here if not directly written in the Readmes.

## External Dependencies
There are several external dependencies needed in order to effectively run the YAM software stack. First, and obviously, ROS 1 needs to be installed. As Python 2 is dying and ROS 1 is soon to follow I expect this installation to become more difficult. Perhaps I will rewrite all of this in ROS 2 in the near future.

Install the following python dependencies
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_Python_SSD1306) ~ Note its deprecated but gets the job done for simple OLED display control.

Then install the following ROS packages
- [MP9250 IMU firmware driver](https://github.com/StefanKrupop/ros-mpu9250-imu) 

Optionally install,
- [ROS Joystick Drivers](https://github.com/ros-drivers/joystick_drivers/tree/main/joy) via `sudo apt install ros-melodic-joystick-drivers` ~ Used for manual control with an Xbox One controller.
- [Joy]() via sudo apt install ros-melodic-joy ~ Joystick interface
- [FaBo9Axis_MPU9250](https://github.com/FaBoPlatform/FaBo9AXIS-MPU9250-Python) via pip install FaBo9Axis_MPU9250 ~ For testing the IMU in a script.

VIO
https://github.com/uzh-rpg/rpg_svo
https://github.com/uzh-rpg/rpg_svo/wiki/Installation:-General-for-ARM-processors

## Fork Notes
I forked this from an alt github that I intend to delete in the near future.
