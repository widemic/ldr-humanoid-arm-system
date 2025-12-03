#!/usr/bin/env python3
"""
Check if MoveIt Task Constructor is installed and available.

This script verifies that all required MTC packages are installed
and provides installation instructions if they are missing.

Author: LDR Robotics Team
License: MIT
"""

import sys

def check_mtc_installation():
    """Check if MTC packages are installed."""
    print("="*60)
    print("MoveIt Task Constructor Installation Check")
    print("="*60)

    missing_packages = []

    # Check for ROS 2 packages
    try:
        import subprocess
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            check=True
        )
        packages = result.stdout

        required_packages = [
            'moveit_task_constructor_core',
            'moveit_task_constructor_msgs',
            'moveit_task_constructor_capabilities',
            'moveit_task_constructor_visualization'
        ]

        print("\nChecking for required ROS 2 packages:")
        for pkg in required_packages:
            if pkg in packages:
                print(f"  ✓ {pkg}")
            else:
                print(f"  ✗ {pkg} - NOT FOUND")
                missing_packages.append(pkg)
    except Exception as e:
        print(f"\nError checking ROS 2 packages: {e}")
        return False

    # Check for Python bindings
    print("\nChecking for Python bindings:")
    try:
        from moveit.task_constructor import core, stages
        print("  ✓ moveit.task_constructor.core")
        print("  ✓ moveit.task_constructor.stages")
    except ImportError as e:
        print(f"  ✗ Python bindings - NOT FOUND")
        print(f"    Error: {e}")
        missing_packages.append("python3-moveit-task-constructor")

    # Results
    print("\n" + "="*60)
    if missing_packages:
        print("STATUS: MTC NOT FULLY INSTALLED")
        print("="*60)
        print("\nMissing components:")
        for pkg in missing_packages:
            print(f"  - {pkg}")

        print("\n" + "="*60)
        print("INSTALLATION INSTRUCTIONS")
        print("="*60)
        print("\nTo install MoveIt Task Constructor for ROS 2 Jazzy:")
        print("\n1. Install from apt:")
        print("   sudo apt update")
        print("   sudo apt install ros-jazzy-moveit-task-constructor-*")

        print("\n2. Source your ROS 2 installation:")
        print("   source /opt/ros/jazzy/setup.bash")

        print("\n3. Rebuild your workspace:")
        print("   cd /path/to/ldr-humanoid-arm-system")
        print("   colcon build --packages-select arm_moveit_config arm_demos")
        print("   source install/setup.bash")

        print("\n4. Run this check again:")
        print("   ros2 run arm_demos check_mtc.py")

        print("\n" + "="*60)
        return False
    else:
        print("STATUS: MTC FULLY INSTALLED ✓")
        print("="*60)
        print("\nYou can now run MTC demos:")
        print("  ros2 launch arm_demos mtc_demo.launch.py demo:=simple")
        print("\nOr run demo scripts directly:")
        print("  ros2 run arm_demos mtc_simple_demo.py")
        print("  ros2 run arm_demos mtc_pick_place_demo.py")
        print("\n" + "="*60)
        return True

def main():
    """Main entry point."""
    success = check_mtc_installation()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
