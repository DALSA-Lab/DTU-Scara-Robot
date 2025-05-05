cmake_minimum_required(VERSION 3.8)
project(joint_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED) # <----custom
find_library(LGPIO_LIBRARY lgpio) # <----custom


add_executable(joint_comm_node src/joint_comm_node.cpp src/uI2C.cpp src/mJoint.cpp src/mJointCom.cpp src/mGripper.cpp)
ament_target_dependencies(joint_comm_node rclcpp) # <----custom

target_link_libraries(joint_comm_node ${LGPIO_LIBRARY}) # <----custom

target_include_directories(joint_comm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(joint_comm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS joint_comm_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
