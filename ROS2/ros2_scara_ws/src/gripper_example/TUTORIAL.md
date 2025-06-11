# Tutorial for controlling the gripper through the robot from a Windows machine

This tutorial is a compilatio of the README files in this repository and specific to just controlling the gripper from a Windows machine.

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
navigate to the ROS workspace:
```bash
cd bioscara/ROS2/ros2_scara_ws
```
Source the ROS executables:
> [!NOTE]
>
> The following step needs only to be done once after opening a new terminal session or when the 'Package 'gripper_example' not found' error is shown.
```bash
source install/local_setup.sh
```

And finally execute the gripper example program:
```bash
ros2 run gripper_example main
```