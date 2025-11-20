cmake_minimum_required(VERSION 3.8)
project(max_accel_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bioscara_hardware_driver REQUIRED)

add_executable(main src/main.cpp)
ament_target_dependencies(main bioscara_hardware_driver)
install(TARGETS main
        DESTINATION lib/${PROJECT_NAME})

ament_package()
