# Bioscara - DALSA DIY SCARA robot arm
This repository serves to collect all information regarding DTU DALSAs DIY robot arm Bioscara version 2.
Version 2 is defined by replacing the MKS SERVO42C stepper drivers with the Ustepper S32 drivers and removing the 3D printer board. The orignal repository can be found in the bioscara_v1 branch.

## Documentation
The documentation for the joint communication interface can be found in [/docs](/docs/). The documentation is made with Doxygen, its configuration file is in the root of the folder. The documentation can be found as a pdf [/docs/latex/refman.pdf](/docs/latex/refman.pdf) and as html in [/docs/html/index.html](/docs/html/index.html)


## TODO
### Mechanical
- [ ]  Update the Fusion model to with all new parts.
- [x] combine old and new stp and stl files
- [ ] Fix J3 Pulley slipping by increasing surface roughness
- [ ] Gripper Redesign
### Software
- [ ] Finish the TODO list for the Joint Communication Protocol (see the documentation)
- [ ] Create URDF file
- [ ] Get started with MoveIt
### Documentation
- [ ] Continue this TODO list and Readme
- [ ] Improve User guide with detailed descriptions and up to date 3D models
- [ ] Document mechanical changes
- [ ] Document Ubuntu install and configurations
- [ ] Document ROS installation
- [ ] Explain the scripts
