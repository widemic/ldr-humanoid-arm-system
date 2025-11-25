cmake_minimum_required(VERSION 3.8)
project(arm_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for MTC
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)

# C++ MTC Pick and Place executable
add_executable(mtc_pick_place src/pick_place_task.cpp)
ament_target_dependencies(mtc_pick_place
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  rclcpp_action
  control_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
)

install(TARGETS
  mtc_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (ROS nodes)
install(PROGRAMS
  scripts/object_recognition_node.py
  scripts/perception_node.py
  scripts/planning_scene_updater.py
  scripts/dynamic_object_tracker.py
  scripts/test_environment_publisher.py
  scripts/test_workspace.py
  scripts/pick_and_place_cylinder.py
  scripts/pick_and_place_action.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install utility modules (not executable as ROS nodes)
# These are Python libraries imported by other scripts:
install(FILES
  scripts/config_loader.py
  scripts/download_yolo_models.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
