cmake_minimum_required(VERSION 3.8)
project(arm_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# MTC Simple Demo (C++)
add_executable(mtc_simple_demo src/mtc_simple_demo.cpp)
ament_target_dependencies(mtc_simple_demo
  rclcpp
  moveit_task_constructor_core
  moveit_core
  geometry_msgs
  tf2_geometry_msgs
)

# MTC Pick Place Demo (C++)
add_executable(mtc_pick_place_demo src/mtc_pick_place_demo.cpp)
ament_target_dependencies(mtc_pick_place_demo
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
)

# Install C++ executables
install(TARGETS
  mtc_simple_demo
  mtc_pick_place_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/check_mtc.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
