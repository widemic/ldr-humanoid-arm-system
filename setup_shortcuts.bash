#!/bin/bash
# ROS 2 LDR Humanoid Arm System - Terminal Shortcuts
# Source this file: source setup_shortcuts.bash

# Get project root directory
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ============================================================================
# Build & Source Shortcuts
# ============================================================================

alias cb='colcon build'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --packages-select'
alias cbv='colcon build --event-handlers console_direct+'
alias cs='source install/setup.bash'
alias ccs='colcon build && source install/setup.bash'
alias clean='rm -rf build/ install/ log/'
alias rebuild='rm -rf build/ install/ log/ && colcon build'

# Build specific package and source
cbps() {
    colcon build --packages-select "$1" && source install/setup.bash
}

# ============================================================================
# ROS 2 Command Shortcuts
# ============================================================================

# Topic commands
alias r2tl='ros2 topic list'
alias r2te='ros2 topic echo'
alias r2th='ros2 topic hz'
alias r2ti='ros2 topic info'

# Node commands
alias r2nl='ros2 node list'
alias r2ni='ros2 node info'

# Launch commands
alias r2l='ros2 launch'
alias r2r='ros2 run'

# Controller commands
alias r2cl='ros2 control list_controllers'
alias r2cs='ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers'

# ============================================================================
# Project-Specific Shortcuts
# ============================================================================

# Navigate to project
alias arm_home='cd '"$PROJECT_ROOT"

# Launch shortcuts
alias arm_gui='ros2 run arm_gui_tools full_system_launcher'
alias arm_full='ros2 launch arm_system_bringup full_system.launch.py'
alias arm_moveit='ros2 launch arm_system_bringup moveit_gazebo.launch.py'
alias arm_gazebo='ros2 launch arm_gazebo arm_world.launch.py'
alias arm_rviz='rviz2 -d $(ros2 pkg prefix arm_perception)/share/arm_perception/config/deep_camera.rviz'

# Launch with world selection
arm_launch() {
    if [ -z "$1" ]; then
        echo "Usage: arm_launch <world_name>"
        echo "Available worlds: lab.sdf, obstacle_course.sdf, pick_place.sdf, manipulation_demo.sdf"
        return 1
    fi
    ros2 launch arm_system_bringup full_system.launch.py simulation_world:="$1"
}

# Test shortcuts
alias arm_test='ros2 run arm_control example.py'
alias arm_test_simple='ros2 run arm_control test_simple.py'

# Controller monitoring
alias arm_controllers='ros2 control list_controllers'
alias arm_joints='ros2 topic echo /joint_states'
alias arm_status='echo "=== Controllers ===" && ros2 control list_controllers && echo -e "\n=== Joint States ===" && ros2 topic hz /joint_states --once'

# Gazebo shortcuts
alias gz_gui='gz sim -g'
alias gz_topics='gz topic -l'
alias gz_models='gz model -l'

# ============================================================================
# Diagnostic Functions
# ============================================================================

# Check if system is ready
arm_check() {
    echo "=== System Health Check ==="
    echo ""
    echo "Controllers:"
    ros2 control list_controllers 2>/dev/null || echo "  ❌ Controller manager not running"
    echo ""
    echo "Active nodes:"
    ros2 node list 2>/dev/null | head -10 || echo "  ❌ No ROS 2 nodes running"
    echo ""
    echo "Joint states:"
    timeout 2 ros2 topic hz /joint_states 2>/dev/null || echo "  ❌ Joint states not publishing"
}

# Watch joint states
arm_watch() {
    ros2 topic echo /joint_states --field name,position
}

# Monitor controller performance
arm_monitor() {
    watch -n 0.5 'ros2 control list_controllers && echo "" && ros2 topic hz /joint_states --once'
}

# ============================================================================
# Development Helpers
# ============================================================================

# Quick rebuild of specific package
arm_rebuild() {
    if [ -z "$1" ]; then
        echo "Usage: arm_rebuild <package_name>"
        echo "Available packages: arm_control, arm_gazebo, arm_moveit_config, arm_perception, etc."
        return 1
    fi
    cd "$PROJECT_ROOT" && \
    colcon build --packages-select "$1" --symlink-install && \
    source install/setup.bash && \
    echo "✅ Package $1 rebuilt and sourced"
}

# Show available commands
arm_help() {
    echo "=== LDR Humanoid Arm System - Terminal Shortcuts ==="
    echo ""
    echo "Build & Source:"
    echo "  cb              - colcon build"
    echo "  cbs             - colcon build --symlink-install"
    echo "  cbp <pkg>       - colcon build --packages-select"
    echo "  cs              - source install/setup.bash"
    echo "  ccs             - build and source"
    echo "  cbps <pkg>      - build package and source"
    echo "  clean           - remove build/install/log"
    echo "  rebuild         - clean build"
    echo ""
    echo "Launch System:"
    echo "  arm_gui         - Launch GUI system launcher (RECOMMENDED)"
    echo "  arm_full        - Launch full system (headless sim + controllers)"
    echo "  arm_launch <w>  - Launch with specific world"
    echo "  arm_moveit      - Launch Gazebo + MoveIt + RViz"
    echo "  arm_gazebo      - Launch Gazebo world only"
    echo "  arm_rviz        - Launch RViz with perception config"
    echo ""
    echo "Testing:"
    echo "  arm_test        - Run example motion"
    echo "  arm_test_simple - Run simple test"
    echo ""
    echo "Monitoring:"
    echo "  arm_check       - System health check"
    echo "  arm_status      - Controller and topic status"
    echo "  arm_controllers - List active controllers"
    echo "  arm_joints      - Echo joint states"
    echo "  arm_watch       - Watch joint positions"
    echo "  arm_monitor     - Live controller monitor"
    echo ""
    echo "Development:"
    echo "  arm_rebuild <pkg> - Rebuild specific package"
    echo "  arm_home        - Navigate to project root"
    echo ""
    echo "ROS 2 Commands:"
    echo "  r2tl, r2te, r2th, r2ti - topic list/echo/hz/info"
    echo "  r2nl, r2ni             - node list/info"
    echo "  r2l, r2r               - launch/run"
    echo "  r2cl, r2cs             - controller list/status"
    echo ""
    echo "Gazebo:"
    echo "  gz_gui          - Launch Gazebo GUI viewer"
    echo "  gz_topics       - List Gazebo topics"
    echo "  gz_models       - List Gazebo models"
}

# ============================================================================
# Initialization
# ============================================================================

# Auto-source workspace if in project directory
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo "✅ Workspace sourced from $PROJECT_ROOT"
else
    echo "⚠️  Workspace not built yet. Run 'cb' to build."
fi

# Show help on first load
echo ""
echo "🤖 LDR Humanoid Arm System shortcuts loaded!"
echo "   Type 'arm_help' to see all available commands"
echo ""
