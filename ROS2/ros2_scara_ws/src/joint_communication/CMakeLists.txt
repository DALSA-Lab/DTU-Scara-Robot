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

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/uI2C.cpp src/mJoint.cpp src/mJointCom.cpp src/mGripper.cpp)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_target_dependencies(${PROJECT_NAME} rclcpp)

# ament_export_dependencies(rclcpp)

target_link_libraries(${PROJECT_NAME} ${LGPIO_LIBRARY})



# ament_target_dependencies(${PROJECT_NAME} rclcpp) # <----custom

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)


install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    # EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

set(RCLCPP_LOCAL_BINARY_NAME joint_communication_node)


add_executable(${RCLCPP_LOCAL_BINARY_NAME} src/joint_comm_node.cpp src/uI2C.cpp src/mJoint.cpp src/mJointCom.cpp src/mGripper.cpp)


ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}

    rclcpp


    )


target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}

    ${PROJECT_NAME}


    )


target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC

    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

    $<INSTALL_INTERFACE:include>)


target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)


install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}

    DESTINATION lib/${PROJECT_NAME})


unset(RCLCPP_LOCAL_BINARY_NAME)


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
