# Robot Gripper  
The gripper is driven by a PWM controlled RC servo. The frequency is 50 Hz. The PWM should be a hardware generated PWM as software PWM has potentiall hight jitter which could cause overheating.
Interacting with the gripper is done through the Gripper class defined in joint_communication/mGripper.h. See the relevant documentation also in the HTML or PDF documentation of the joint_communication protocol.  

## Usage
After building and sourcing execute the program with the following command:
`ros2 run gripper_example main`
