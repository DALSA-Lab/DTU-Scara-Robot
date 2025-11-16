cmake_minimum_required(VERSION 3.8)
project(bioscara_gripper_hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bioscara_arm_hardware_driver REQUIRED)

# Making the Library
add_library(
  ${PROJECT_NAME} 
  SHARED 
  src/mBaseGripper.cpp
  src/mMockGripper.cpp
  src/mGripper.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(
  ${PROJECT_NAME}
  bioscara_arm_hardware_driver
)

# Installing library 
## register header files to make the available to other packages using this lib
install(
  DIRECTORY include/
  DESTINATION include/#${PROJECT_NAME} ## <--- check this again
)

## install targets and create export target
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  bioscara_arm_hardware_driver
)

ament_package()