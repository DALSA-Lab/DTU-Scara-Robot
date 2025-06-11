# ROS2 Workspace
This workspace contains all ROS2 packages needed to operate the robot. Below two custom packages are listed. Each custom package contains a README in the */src* directory, describing the the package in greater detail.

## Usage
To run a package it needs to be build. Navigate to the ROS2 workspace *ROS2/ros2_scara_ws* and execute `colcon build --packages-select <package_name>`. Additionally source the packages: `source install/local_setup.sh`

### joint_communication
This package contains the core-API for interacting with the joint controllers via I2C and the PWM controlled gripper.

### gripper_example
This package showcases a basic interactive control of the robot gripper.
