# Bioscara - DALSA DIY SCARA robot arm
This repository serves to collect all information regarding DTU DALSAs DIY robot arm Bioscara version 2.
Version 2 is defined by replacing the MKS SERVO42C stepper drivers with the Ustepper S32 drivers and removing the 3D printer board. The orignal repository can be found in the bioscara_v1 branch.

## Documentation
The documentation for the joint communication interface can be found in [/docs](/docs/). The documentation is made with Doxygen, its configuration file is in the root of the folder. The documentation can be found as a pdf [/docs/latex/refman.pdf](/docs/latex/refman.pdf) and as html in [/docs/html/index.html](/docs/html/index.html)

## Usage
The robots controller is a Raspberry Pi 4. The RPI is configured to connect to a WIFI network with the the following credentials:  
**SSID**: DALSA_IOT  
**Password**: dalsa_iot  
The easiest way to establish the network is to create a WIFI hotspot with the above credentials, however static IP assignemnt is not possible using Windows Hotspot, and hence the IP address must be checked before any connection attempt.
On the RPI remote login and remote desktop as well as SSH is activated. This way the desktop can be accessed through a RDP software like Windows Remote Desktop or Remina. Be carefull that connecting this way will create a new user session!

Interacting with the robot is handled through ROS2. In the subdirectory a seperate README can be found explaining how to use the hardware.

## TODO
### Mechanical
- [ ]  Update the Fusion model to with all new parts.
- [x] combine old and new stp and stl files
- [ ] Fix J3 Pulley slipping by increasing surface roughness
- [ ] Gripper Redesign
### Software
- [ ] Finish the TODO list for the Joint Communication Protocol (see the documentation)
### Documentation
- [ ] Improve User guide with detailed descriptions and up to date 3D models
- [ ] Document mechanical changes
- [ ] Document Ubuntu install and configurations
- [ ] Explain the scripts
