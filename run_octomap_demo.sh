#!/bin/bash
# Complete OctoMap Demo Script
# Runs octomap_server + RViz with native displays

set -e

echo "========================================="
echo "OctoMap Complete Setup Demo"
echo "========================================="
echo ""

# Change to workspace
cd ~/ros2_ws/ldr-humanoid-arm-system

# Check if we need to build first
if [ ! -d "install" ]; then
    echo "❌ Workspace not built. Run 'colcon build' first."
    exit 1
fi

echo "✓ Workspace found"
echo ""

# Source workspaces
echo "Step 1: Sourcing workspaces..."
source install/setup.bash
echo "  ✓ Main workspace sourced"

# Check if overlay exists
if [ -d ~/ros2_ws/octomap_overlay_ws/install ]; then
    source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
    echo "  ✓ OctoMap overlay workspace sourced (native displays enabled)"
    NATIVE_DISPLAYS=true
else
    echo "  ⚠ OctoMap overlay not found (native displays disabled)"
    echo "    Run ./rebuild_octomap_plugin.sh to enable OccupancyMap displays"
    NATIVE_DISPLAYS=false
fi
echo ""

# Check dependencies
echo "Step 2: Checking dependencies..."
if ! ros2 pkg list | grep -q "octomap_server"; then
    echo "❌ octomap_server not installed"
    echo "   Install with: sudo apt install ros-jazzy-octomap-server"
    exit 1
fi
echo "  ✓ octomap_server installed"

if ! command -v octovis &> /dev/null; then
    echo "  ⚠ octovis not installed (optional)"
    echo "    Install with: sudo apt install octovis"
else
    echo "  ✓ octovis installed"
fi
echo ""

# Check configuration files exist
echo "Step 3: Verifying configuration files..."
CONFIG_FILES=(
    "src/bringup/arm_system_bringup/launch/octomap_server.launch.py"
    "src/bringup/arm_system_bringup/config/octomap_server.yaml"
    "src/planning/arm_moveit_config/config/moveit_with_octomap.rviz"
    "src/bringup/arm_system_bringup/launch/moveit_gazebo_with_octomap.launch.py"
)

for file in "${CONFIG_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        echo "❌ Missing: $file"
        exit 1
    fi
    echo "  ✓ $(basename $file)"
done
echo ""

# Display launch options
echo "========================================="
echo "Launch Options:"
echo "========================================="
echo ""
echo "Choose how to run the demo:"
echo ""
echo "1) Integrated Launch (Recommended - All-in-One)"
echo "   Single command launches everything"
echo ""
echo "2) Manual Launch (Step-by-Step)"
echo "   Launch components separately in multiple terminals"
echo ""
echo "3) Just RViz (System already running)"
echo "   Only launch RViz with OctoMap config"
echo ""
read -p "Enter choice (1-3): " choice

case $choice in
    1)
        echo ""
        echo "========================================="
        echo "INTEGRATED LAUNCH"
        echo "========================================="
        echo ""
        echo "Launching complete system with OctoMap..."
        echo ""
        echo "Timeline:"
        echo "  T=0s   : Gazebo + Robot + Controllers"
        echo "  T=10s  : OctoMap Server"
        echo "  T=12s  : RViz with OctoMap displays"
        echo ""
        echo "Expected displays in RViz:"
        echo "  - Grid"
        echo "  - MotionPlanning"
        echo "  - Camera_PointCloud (colorful sensor data)"
        echo "  - OctoMap_Voxels (red squares - occupied space)"
        echo ""
        if [ "$NATIVE_DISPLAYS" = true ]; then
            echo "Native displays available! In RViz you can also add:"
            echo "  - Add → By topic → /octomap_binary → OccupancyMap (colorful 3D voxels)"
            echo ""
        fi
        echo "Press Ctrl+C to stop when done"
        echo ""
        read -p "Press Enter to launch..."

        ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
        ;;

    2)
        echo ""
        echo "========================================="
        echo "MANUAL LAUNCH"
        echo "========================================="
        echo ""
        echo "This will open multiple terminals automatically."
        echo ""
        echo "Terminal 1: Gazebo + Controllers + MoveIt"
        echo "Terminal 2: OctoMap Server"
        echo "Terminal 3: RViz"
        echo ""
        read -p "Press Enter to start..."

        # Check if we can use gnome-terminal
        if command -v gnome-terminal &> /dev/null; then
            echo "Opening terminals..."

            # Terminal 1: Main system
            gnome-terminal -- bash -c "
                cd ~/ros2_ws/ldr-humanoid-arm-system
                source install/setup.bash
                echo '=== Terminal 1: Launching Gazebo + MoveIt ==='
                echo 'Wait for Gazebo to fully load (~20 seconds)...'
                ros2 launch arm_system_bringup moveit_gazebo.launch.py
                exec bash
            " &

            sleep 2

            # Terminal 2: OctoMap server
            gnome-terminal -- bash -c "
                echo 'Waiting 25 seconds for main system to initialize...'
                sleep 25
                cd ~/ros2_ws/ldr-humanoid-arm-system
                source install/setup.bash
                echo ''
                echo '=== Terminal 2: Launching OctoMap Server ==='
                ros2 launch arm_system_bringup octomap_server.launch.py use_sim_time:=true
                exec bash
            " &

            sleep 2

            # Terminal 3: RViz
            gnome-terminal -- bash -c "
                echo 'Waiting 30 seconds for OctoMap to start...'
                sleep 30
                cd ~/ros2_ws/ldr-humanoid-arm-system
                source install/setup.bash
                if [ -d ~/ros2_ws/octomap_overlay_ws/install ]; then
                    source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
                    echo ''
                    echo '=== Terminal 3: Launching RViz (Native Displays Enabled) ==='
                else
                    echo ''
                    echo '=== Terminal 3: Launching RViz ==='
                fi
                rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz
                exec bash
            " &

            echo ""
            echo "✓ Terminals launched!"
            echo ""
            echo "Monitor the terminals for status messages."
            echo "RViz should open automatically in ~30 seconds."
            echo ""

        else
            echo ""
            echo "gnome-terminal not found. Manual terminal launch required."
            echo ""
            echo "Open 3 terminals and run these commands:"
            echo ""
            echo "--- Terminal 1 ---"
            echo "cd ~/ros2_ws/ldr-humanoid-arm-system"
            echo "source install/setup.bash"
            echo "ros2 launch arm_system_bringup moveit_gazebo.launch.py"
            echo ""
            echo "--- Terminal 2 (wait 25s) ---"
            echo "cd ~/ros2_ws/ldr-humanoid-arm-system"
            echo "source install/setup.bash"
            echo "ros2 launch arm_system_bringup octomap_server.launch.py use_sim_time:=true"
            echo ""
            echo "--- Terminal 3 (wait 30s) ---"
            echo "cd ~/ros2_ws/ldr-humanoid-arm-system"
            echo "source install/setup.bash"
            if [ "$NATIVE_DISPLAYS" = true ]; then
                echo "source ~/ros2_ws/octomap_overlay_ws/install/setup.bash"
            fi
            echo "rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz"
            echo ""
        fi
        ;;

    3)
        echo ""
        echo "========================================="
        echo "RVIZ ONLY"
        echo "========================================="
        echo ""
        echo "Launching RViz with OctoMap configuration..."
        echo ""
        if [ "$NATIVE_DISPLAYS" = true ]; then
            echo "Native displays enabled!"
            echo "You can add: OccupancyMap, OccupancyGrid displays"
        fi
        echo ""

        rviz2 -d src/planning/arm_moveit_config/config/moveit_with_octomap.rviz
        ;;

    *)
        echo "Invalid choice"
        exit 1
        ;;
esac
