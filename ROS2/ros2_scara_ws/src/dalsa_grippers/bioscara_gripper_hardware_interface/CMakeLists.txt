cmake_minimum_required(VERSION 3.8)
project(bioscara_gripper_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bioscara_gripper_hardware_driver REQUIRED)



add_library(
  ${PROJECT_NAME}
  SHARED
  src/gripper_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  bioscara_gripper_hardware_driver
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  bioscara_gripper_hardware_driver
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()