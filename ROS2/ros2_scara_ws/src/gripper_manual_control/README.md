# Robot Gripper  
The gripper is driven by a PWM controlled RC servo. The frequency is 50 Hz. The PWM should be a hardware generated PWM as software PWM has potentiall hight jitter which could cause overheating.
<!-- Interacting with the gripper is done through the Gripper class defined in joint_communication/mGripper.h. See the relevant documentation also in the HTML or PDF documentation of the joint_communication protocol.   -->

## Usage
First build the package with Colcon, then you can run it after it is sourced.
### Building
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
colcon build --packages-select gripper_manual_control
```
### Running
In a new terminal:
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
source install/local_setup.sh
ros2 run gripper_example manual_control 
 ```

Sourcing only is necessary once after opening a new terminal.
