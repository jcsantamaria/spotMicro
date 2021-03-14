# Spot Micro Quadruped Project

* [Overview](#Overview)
* [General Instructions](#general-instructions)
* [Description of ROS Nodes](#description-of-ros-nodes)
* [Future Work](#future-work)
* [External Links](#external-links)

## Overview
This project contains a description of my implementation of the Spot Micro quadruped robot: https://spotmicroai.readthedocs.io/en/latest/

The source code is based on https://github.com/mike4192/spotMicro. Many thanks to mike4192!!

The frame of the Spot Micro is based on the 3D print model on https://github.com/michaelkubina/SpotMicroESP32.  Many thanks to Michael Kubina!!

The software is implemented on a Raspberry Pi Zero computer running Raspbian GNU/Linux 10 (buster).

The software is composed of C++ and python nodes in a ROS framework.

### Hardware:
The frame utilized is the variation created by Michael Kubina.  I chose this variation due to: (a) the capability of 3D printing the pieces without supports, (b) the great assembly instructions given by the author, and (c) the capability of the Spot Micro to fully collapse the legs. See https://github.com/michaelkubina/SpotMicroESP32 for all the details on assembly.  I designed and 3D printed my own version of the circuit plate to accomodate different selection of some of the components.

Component List:
* Computer: Raspberry Pi Zero
* Servo control board: PCA9685, controlled via i2c
* Servos: 12 x ANNIMOS DS3218 Digital RC Servos
* IMU: SparkFun VR IMU Breakout - BNO080
* LCD Panel: 16x2 i2c LCD panel
* Battery: 7.4V Lipo Battery 2S 50C 5200mAh
* Voltage Regulator: Anmbest Constant Current CC CV Buck Converter Module DC 6-40V to 1.2-36V 20A 300W. This regulator feeds the servos. Converts battery voltage to 6.2V.
* Buck: MCIGICM LM2596 Buck Converter, DC to DC 3.0-40V to 1.5-35V Step Down Power Supply High Efficiency Voltage Regulator Module.  This regulator feeds the Raspberry Pi Zero and rest of components. Converts 6.2V to 5V.

Servos are connected in the following order to the PCA 9685 control board:
1. Right front knee
2. Right front shoulder
3. Right front hip
4. Right back knee
5. Right back shoulder
6. Right back hip
7. Left back knee
8. Left back shoulder
9. Left back hip
10. Left front knee
11. Left front shoulder
12. left front hip

#### OS:
The OS running on the Raspberry PI Zero is Raspbian Buster.  I installed ROS Kinetic, but there isn't an image directly available for the Raspberry PI Zero. So, I had to build Kinetic from source as described in here: http://wiki.ros.org/Installation/Source.  Remember to increase the swap space of the OS to allow builds to execute successfuly. See https://nebl.io/neblio-university/enabling-increasing-raspberry-pi-swap/.

#### Software:
This repo is a clone of mike4192 Spot Micro Project found here: https://github.com/mike4192/spotMicro.  Mike provides excellent documentation and description of the software structure and layout.  I added these additional ROS nodes to support additional/different hardware components:

- IMU: Suppor to read orientation of the Spot Micro's torso. This ROS node interfaces with the BNO080.

## External Links and References
Spot Micro AI community: https://gitlab.com/custom_robots/spotmicroai
