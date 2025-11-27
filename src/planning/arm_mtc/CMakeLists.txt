cmake_minimum_required(VERSION 3.8)
project(arm_mtc)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# MTC Node executable (original complex version)
add_executable(mtc_node src/mtc_node.cpp)

ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# MTC Node 2 executable (simple tutorial version)
add_executable(mtc_node2 src/mtc_node2.cpp)

ament_target_dependencies(mtc_node2
  rclcpp
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# MTC Pick Place Cylinder executable (complete pick and place demo)
add_executable(mtc_pick_place_cylinder src/mtc_pick_place_cylinder.cpp)

 ament_target_dependencies(mtc_pick_place_cylinder
  rclcpp
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  mtc_node
  mtc_node2
  mtc_pick_place_cylinder
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/simple_pick.py
  scripts/mtc_orchestrator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
