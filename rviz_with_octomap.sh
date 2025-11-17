#!/bin/bash
# Launch RViz with the rebuilt OctoMap plugin
# This ensures the overlay plugin is loaded instead of the broken ROS one

cd ~/ros2_ws/ldr-humanoid-arm-system

# Source workspaces in correct order
source install/setup.bash

# Source overlay workspace with rebuilt plugin
if [ -d ~/ros2_ws/octomap_overlay_ws/install ]; then
    source ~/ros2_ws/octomap_overlay_ws/install/setup.bash
    echo "✓ Overlay workspace sourced - Native OctoMap displays enabled"
else
    echo "⚠ Overlay workspace not found at ~/ros2_ws/octomap_overlay_ws"
    echo "  Run ./rebuild_octomap_plugin.sh to enable native displays"
    echo "  Continuing with PointCloud2 displays only..."
fi

# Force the overlay plugin to load first by prepending to library path
export LD_LIBRARY_PATH="$HOME/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib:$LD_LIBRARY_PATH"

# Also set plugin path explicitly
export RVIZ_PLUGIN_PATH="$HOME/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib:$RVIZ_PLUGIN_PATH"

echo ""
echo "=== Library paths configured ==="
echo "Plugin will load from: $HOME/ros2_ws/octomap_overlay_ws/install/octomap_rviz_plugins/lib"
echo ""

# Check if config file exists
RVIZ_CONFIG="src/planning/arm_moveit_config/config/moveit_with_octomap.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "⚠ RViz config not found: $RVIZ_CONFIG"
    echo "  Launching RViz without config..."
    rviz2 "$@"
else
    echo "Loading RViz config: $RVIZ_CONFIG"
    echo ""
    rviz2 -d "$RVIZ_CONFIG" "$@"
fi
