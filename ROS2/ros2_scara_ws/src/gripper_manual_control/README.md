# Robot Gripper  
The gripper is driven by a PWM controlled RC servo. The frequency is 50 Hz. The PWM should be a hardware generated PWM as software PWM has potentiall hight jitter which could cause overheating.
<!-- Interacting with the gripper is done through the Gripper class defined in joint_communication/mGripper.h. See the relevant documentation also in the HTML or PDF documentation of the joint_communication protocol.   -->

## Usage
This package contains a simple programm that allows to manually control the gripper actuator. Instead of setting a desired width, the actuator angle is set directly between -90.0 and +90.0 degrees. This can be used for testing, positioning when mounting and dismounting and to calculate reduction and offset (explained [here](#calculating-reduction-and-offset)).

First build the package with Colcon, then you can run it after it is sourced. 
### Building
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
colcon build --packages-select gripper_manual_control bioscara_gripper_hardware_driver
```
### Running
In a new terminal:
```bash
cd ~/bioscara/ROS2/ros2_scara_ws
source install/local_setup.sh
ros2 run gripper_example manual_control 
 ```

Sourcing only is necessary once after opening a new terminal.

## Calculating reduction and offset
The gripper has the reduction $r$ and offset $o$ parameters which are used to translate from a desired gripper width to the actuator angle. The relationship between gripper width $w$ and acutator angle $\alpha$ is as follows:
$$
\alpha = r (w-o)
$$

To determine these parameters execute the following steps:

1. Manually set the gripper to an open position by setting a actuator angle. Be carefull to not exceed the physical limits of the gripper since the actuator is strong enough to break PLA before stalling.
2. Measure the gripper width $w_1$ and note the set actuator angle $\alpha_1$.
3. Move the gripper to a more closed position that still allows you to accurately measure the width
4. Measure the second width $w_2$ and note the corresponding angle $\alpha_2$
5. Calculate the offset $o$:
$$
o = \frac{\alpha_1 w_2 -  \alpha_2 w_1}{\alpha_1 - \alpha_2}
$$
6. Calculate the reduction $r$:
$$
r = \frac{\alpha_1}{w_1 - o}
$$