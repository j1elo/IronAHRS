IronAHRS
========

A custom Attitude and Heading Reference System (AHRS) built on a SOC Robotics IMU6410 development board. This was a small step on our personal "Virtual Reality head-mounted display" technology project.

See it in action:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=CbZY3lHCA_A"
target="_blank"><img src="http://img.youtube.com/vi/CbZY3lHCA_A/0.jpg" 
alt="Playing a FPS game with an IMU/AHRS Attitude Tracker"
width="240" height="180" border="10" /></a>

This project serves as an integration and test bench for different IMU and AHRS algorithms. Currently, the incorporated implementation of signal filtering is the one made by Sparkfun for their *9DOF Razor IMU* board: the [Razor AHRS firmware](https://github.com/ptrbrtz/razor-9dof-ahrs).

Other algorithms which could be integrated in the future includes the implementation from Android, the sensor fusion algorithm [made by Sebastian Madgwick](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/), and the [FreeIMU implementation](http://www.varesano.net/topic/freeimu).

Folder structure:
  
- `src/avr` includes all the source code for the firmware which will be loaded on the board's microcontroller.
- `src/avr/board` contains a modularized skeleton of code used to handle the most common functionalities of the Atmel AVR uC used for the project: board initialization, access to the EEPROM, SPI channels, Serial Port communication and time measurement functions.
- `src/razor` contains the implementation of the different AHRS filtering algorithms. This code is shared between the board C code and the application C++ code.
- `src/win` contains the desktop application which serves as a test bench for the sensor board. This application can receive the sensor inputs in a variety of ways, either in RAW form (from the sensors output) or on calculated AHRS positions, and store them for further processing.
