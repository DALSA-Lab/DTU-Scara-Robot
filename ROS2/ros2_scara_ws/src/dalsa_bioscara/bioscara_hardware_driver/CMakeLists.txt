cmake_minimum_required(VERSION 3.8)
project(bioscara_hardware_driver)

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
# find_package(rclcpp REQUIRED) #???
find_library(LGPIO_LIBRARY lgpio)


# Making the Library
add_library(
  ${PROJECT_NAME} 
  SHARED 
  src/uI2C.cpp src/mJoint.cpp src/mJointCom.cpp src/mGripper.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)


# create dependecy to LGPIO
target_link_libraries(${PROJECT_NAME} ${LGPIO_LIBRARY})

# create dependecy to rclcpp
# ament_target_dependencies(
#   ${PROJECT_NAME}
#   rclcpp
# )


# Making the executable
## Set executable name (target name)
set(RCLCPP_LOCAL_BINARY_NAME ${PROJECT_NAME}_node)

## add source files to be compiled for the executable
add_executable(
  ${RCLCPP_LOCAL_BINARY_NAME} 
  src/joint_comm_node.cpp# src/uI2C.cpp src/mJoint.cpp src/mJointCom.cpp src/mGripper.cpp
)

## also link the executable to the library created in this package
target_link_libraries(
  ${RCLCPP_LOCAL_BINARY_NAME}
  ${PROJECT_NAME}
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
# ament_export_dependencies(lgpio) # <-- what to use here? lgpio?


# Do I need this?

# ament_export_include_directories(
#   include
# )
# ament_export_libraries(
#   ${PROJECT_NAME}
# )




# Install executable
install(
  TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

unset(RCLCPP_LOCAL_BINARY_NAME)

ament_package()