cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Build the controller plugin library
add_library(${PROJECT_NAME} SHARED
  src/arm_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  rclcpp_action
)

# Export the plugin
pluginlib_export_plugin_description_file(controller_interface arm_controller_plugin.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/motion_planner.py
  scripts/example.py
  scripts/moveit_cartesian_example.py
  scripts/complete_depth_camera.py
  scripts/power_calculator.py
  scripts/power_monitor_node.py
  scripts/test_power_monitor.py
  scripts/diagnose_jitter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies and targets
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  rclcpp_action
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
