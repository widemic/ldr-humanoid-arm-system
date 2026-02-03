#!/usr/bin/env python3
"""
Simple joint position monitor using PyQt5 with separate UI file.
Displays current joint positions using progress bars.
"""

import sys
import os
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointMonitorNode(Node):
    """ROS 2 node for subscribing to joint states."""

    def __init__(self, gui_callback):
        super().__init__('joint_monitor_gui')
        self.gui_callback = gui_callback

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Joint Monitor GUI started')

    def joint_state_callback(self, msg):
        """Callback for joint state messages."""
        # Create a dictionary of joint positions
        joint_positions = dict(zip(msg.name, msg.position))
        self.gui_callback(joint_positions)


class JointMonitorGUI(QtWidgets.QMainWindow):
    """Main GUI window for joint position monitoring."""

    # Joint name mapping to progress bars
    JOINT_MAPPING = {
        'shoulder_pitch_joint': 'progress_base',
        'shoulder_roll_joint': 'progress_shoulder',
        'shoulder_yaw_joint': 'progress_elbow',
        'elbow_pitch_joint': 'progress_wrist_pitch',
        'elbow_yaw_joint': 'progress_wrist_roll',
    }

    def __init__(self):
        super().__init__()

        # Load UI file from share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = os.path.join(pkg_share, 'ui', 'joint_monitor.ui')
        except:
            # Fallback for development (when running directly)
            ui_file = os.path.join(
                os.path.dirname(__file__),
                '../../ui/joint_monitor.ui'
            )

        uic.loadUi(ui_file, self)

        # Store progress bar widgets
        self.progress_bars = {
            joint: self.findChild(QtWidgets.QProgressBar, progress_name)
            for joint, progress_name in self.JOINT_MAPPING.items()
        }

    def update_joint_positions(self, joint_positions):
        """Update progress bars with current joint positions."""
        for joint_name, progress_bar in self.progress_bars.items():
            if joint_name in joint_positions:
                # Convert radians to progress bar scale (x100 for precision)
                position_rad = joint_positions[joint_name]
                progress_bar.setValue(int(position_rad * 100))

                # Update format string to show actual radians
                progress_bar.setFormat(f'{position_rad:.3f} rad')


def main(args=None):
    """Main entry point."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create GUI
    gui = JointMonitorGUI()
    gui.show()

    # Create ROS node with GUI callback
    node = JointMonitorNode(gui.update_joint_positions)

    # Setup timer to spin ROS node
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)  # 50ms = 20Hz update rate

    # Run Qt event loop
    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
