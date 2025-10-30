cmake_minimum_required(VERSION 3.16)
project(homing_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(control_msgs REQUIRED)

generate_parameter_library(homing_controller_parameters
  src/homing_controller_parameters.yaml
)

add_library(homing_controller
  SHARED
  src/homing_controller.cpp
)
target_include_directories(homing_controller PRIVATE include)
target_link_libraries(homing_controller PUBLIC
                      homing_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})

pluginlib_export_plugin_description_file(controller_interface homing_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  homing_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  homing_controller
)
ament_package()
