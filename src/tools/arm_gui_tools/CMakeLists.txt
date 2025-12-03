cmake_minimum_required(VERSION 3.8)
project(arm_gui_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/${PROJECT_NAME}/power_monitor_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME power_monitor_gui
)

install(PROGRAMS
  src/${PROJECT_NAME}/joint_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_monitor
)

install(PROGRAMS
  src/${PROJECT_NAME}/full_system_launcher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME full_system_launcher
)

install(DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
