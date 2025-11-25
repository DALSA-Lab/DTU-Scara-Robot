cmake_minimum_required(VERSION 3.8)
project(gripper_manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bioscara_gripper_hardware_driver REQUIRED)

add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(manual_control bioscara_gripper_hardware_driver)
install(TARGETS manual_control
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  bioscara_gripper_hardware_driver
)
ament_package()
