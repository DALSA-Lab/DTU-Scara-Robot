# Documentation
This documentation currently documents how the robot controller communicates with the joint controllers, this includes:
- The joint firmware in the [/Arduino](../Arduino/) directory
- The interfacing library used for communicating with the joints in the [/ROS2](../ROS2/ros2_scara_ws/src/joint_communication/) directory.

# Usage
the joint_communication library is structured as a ROS2 package but can also be used in another build toolchain. If that is the case ensure the include paths are still correct.

