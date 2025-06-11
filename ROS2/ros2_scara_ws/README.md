## Usage
To run a package it needs to be build. Navigate to the ROS2 workspace *ROS2/ros2_scara_ws* and execute `colcon build --packages-select <package_name>`. Additionally source the packages: `source install/local_setup.sh`

### joint_communication
This package contains the core-API for interacting with the joint controllers via I2C and the PWM controlled gripper.

### gripper_example
This package showcases a basic interactive control of the robot gripper.
