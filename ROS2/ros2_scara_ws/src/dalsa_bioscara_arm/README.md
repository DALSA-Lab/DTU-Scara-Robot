The packages are structured according to this guide:
 [RTW Package Structure](https://rtw.b-robotized.com/master/guidelines/robot_package_structure.html)


 When compiling the package is installed in the share/ directory. Also the URDF is stored there. The bioscara.launch.py file expects to find the urdf there. This is done in the packages cmake file

```cmake
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_1
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_1
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_1
)
install(TARGETS ros2_control_demo_example_1
  EXPORT export_ros2_control_demo_example_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

TODO:
- [ ] Format and rework this content