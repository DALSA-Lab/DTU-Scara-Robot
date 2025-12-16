# Tutorial for controlling the gripper through the robot from a Windows machine

This tutorial is a compilation of the README files in this repository and specific to just controlling the gripper from a Windows machine.

## Network
After turning on the robot, the Raspberry Pi will search for a WIFI network with the the following credentials:  
**SSID**: DALSA_IOT  
**Password**: dalsa_iot  

Create this using the Windows Mobile Hotspot.
In the hotspot settings you will see when the RPI has connected and its IP adress.

## Logging in Remotly
To execute programs on the RPI you have to log in remotely using SSH. On Windows open the command prompt terminal by searching for "cmd" in the start menu.

In the terminal log in to the RPI by establishing a SSH connection as follows:
```bash
ssh scara@<ip-adress>
```
Type the password **dtubio** when prompted and hit enter.

## Executing the gripper control example program
When connected to the robot controller via SSH follow the instructions in the README to execute the manual control program.