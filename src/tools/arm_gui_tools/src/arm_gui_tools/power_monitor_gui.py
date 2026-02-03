#!/usr/bin/env python3
"""
PyQt5 GUI for real-time power and current consumption monitoring.

Displays:
- Total system power and current
- Per-joint power consumption (bar charts)
- Real-time power graph
- Statistics (peak, average, cumulative energy)
- Battery life estimation
"""

import sys
import os
from collections import deque
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QProgressBar, QPushButton, QGridLayout,
                               QGroupBox, QLineEdit, QCheckBox, QTextEdit)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

# Import power calculator
sys.path.append('/home/andrei-dragomir/Documents/GitHub/ldr-humanoid-arm-system/src/control/arm_control/scripts')
from power_calculator import PowerCalculator


class PowerMonitorROSNode(Node):
    """ROS 2 node for power monitoring."""

    def __init__(self, gui_callback):
        super().__init__('power_monitor_gui_node')
        self.gui_callback = gui_callback

        # Load power calculator
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_control')
            specs_path = os.path.join(pkg_share, 'config', 'actuator_specs.yaml')
        except:
            specs_path = '/home/andrei-dragomir/Documents/GitHub/ldr-humanoid-arm-system/src/control/arm_control/config/actuator_specs.yaml'

        self.power_calc = PowerCalculator(specs_path)
        self.latest_joint_states = {}

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Power Monitor GUI ROS node started')

    def joint_state_callback(self, msg: JointState):
        """Process joint states and calculate power."""
        # Update joint states
        for i, name in enumerate(msg.name):
            self.latest_joint_states[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0
            }

        # Calculate power
        if self.latest_joint_states:
            result = self.power_calc.calculate_total_power(self.latest_joint_states)
            self.gui_callback(result)


class PowerPlot(FigureCanvasQTAgg):
    """Real-time power consumption plot."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(PowerPlot, self).__init__(fig)

        self.setParent(parent)

        # Data storage (keep last 100 points)
        self.max_points = 100
        self.time_data = deque(maxlen=self.max_points)
        self.power_data = deque(maxlen=self.max_points)
        self.current_data = deque(maxlen=self.max_points)

        self.start_time = 0

        # Initialize plot
        self.axes.set_xlabel('Time (s)')
        self.axes.set_ylabel('Power (W)', color='tab:blue')
        self.axes.tick_params(axis='y', labelcolor='tab:blue')
        self.axes.grid(True, alpha=0.3)

        self.power_line, = self.axes.plot([], [], 'b-', label='Power (W)', linewidth=2)

        # Twin axis for current
        self.axes2 = self.axes.twinx()
        self.axes2.set_ylabel('Current (A)', color='tab:orange')
        self.axes2.tick_params(axis='y', labelcolor='tab:orange')

        self.current_line, = self.axes2.plot([], [], 'orange', label='Current (A)', linewidth=2)

        # Legend
        lines = [self.power_line, self.current_line]
        labels = [l.get_label() for l in lines]
        self.axes.legend(lines, labels, loc='upper left')

        self.draw()

    def update_plot(self, time_s, power_w, current_a):
        """Update plot with new data point."""
        if self.start_time == 0:
            self.start_time = time_s

        rel_time = time_s - self.start_time

        self.time_data.append(rel_time)
        self.power_data.append(power_w)
        self.current_data.append(current_a)

        # Update plot data
        self.power_line.set_data(list(self.time_data), list(self.power_data))
        self.current_line.set_data(list(self.time_data), list(self.current_data))

        # Auto-scale
        if len(self.time_data) > 0:
            self.axes.set_xlim(min(self.time_data), max(self.time_data) + 1)
            if len(self.power_data) > 0:
                p_max = max(self.power_data)
                self.axes.set_ylim(0, p_max * 1.1 if p_max > 0 else 100)
            if len(self.current_data) > 0:
                c_max = max(self.current_data)
                self.axes2.set_ylim(0, c_max * 1.1 if c_max > 0 else 10)

        self.draw()


class PowerMonitorGUI(QMainWindow):
    """Main GUI window for power monitoring."""

    def __init__(self):
        super().__init__()

        self.setWindowTitle('Robot Power Monitor')
        self.setGeometry(100, 100, 1200, 800)

        # Statistics
        self.total_energy_wh = 0.0
        self.peak_power_w = 0.0
        self.avg_power_w = 0.0
        self.sample_count = 0
        self.last_update_time = 0

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Left panel: Statistics and controls
        left_panel = QVBoxLayout()

        # Total power display
        power_group = QGroupBox("System Power")
        power_layout = QGridLayout()

        self.total_power_label = QLabel("0.0 W")
        self.total_power_label.setStyleSheet("font-size: 32px; font-weight: bold; color: #2196F3;")
        self.total_power_label.setAlignment(Qt.AlignCenter)

        self.total_current_label = QLabel("0.0 A")
        self.total_current_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #FF9800;")
        self.total_current_label.setAlignment(Qt.AlignCenter)

        power_layout.addWidget(QLabel("Total Power:"), 0, 0)
        power_layout.addWidget(self.total_power_label, 0, 1)
        power_layout.addWidget(QLabel("Total Current:"), 1, 0)
        power_layout.addWidget(self.total_current_label, 1, 1)

        power_group.setLayout(power_layout)
        left_panel.addWidget(power_group)

        # Statistics display
        stats_group = QGroupBox("Statistics")
        stats_layout = QGridLayout()

        self.peak_power_label = QLabel("0.0 W")
        self.avg_power_label = QLabel("0.0 W")
        self.energy_label = QLabel("0.000 Wh")
        self.efficiency_label = QLabel("0.0 %")

        stats_layout.addWidget(QLabel("Peak Power:"), 0, 0)
        stats_layout.addWidget(self.peak_power_label, 0, 1)
        stats_layout.addWidget(QLabel("Average Power:"), 1, 0)
        stats_layout.addWidget(self.avg_power_label, 1, 1)
        stats_layout.addWidget(QLabel("Total Energy:"), 2, 0)
        stats_layout.addWidget(self.energy_label, 2, 1)
        stats_layout.addWidget(QLabel("Efficiency:"), 3, 0)
        stats_layout.addWidget(self.efficiency_label, 3, 1)

        stats_group.setLayout(stats_layout)
        left_panel.addWidget(stats_group)

        # Battery estimation
        battery_group = QGroupBox("Battery Life Estimation")
        battery_layout = QGridLayout()

        battery_layout.addWidget(QLabel("Battery Capacity (Ah):"), 0, 0)
        self.battery_capacity_input = QLineEdit("20.0")
        battery_layout.addWidget(self.battery_capacity_input, 0, 1)

        battery_layout.addWidget(QLabel("Battery Voltage (V):"), 1, 0)
        self.battery_voltage_input = QLineEdit("48.0")
        battery_layout.addWidget(self.battery_voltage_input, 1, 1)

        self.battery_life_label = QLabel("-- hours")
        self.battery_life_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #4CAF50;")
        battery_layout.addWidget(QLabel("Estimated Runtime:"), 2, 0)
        battery_layout.addWidget(self.battery_life_label, 2, 1)

        battery_group.setLayout(battery_layout)
        left_panel.addWidget(battery_group)

        # Controls
        control_group = QGroupBox("Controls")
        control_layout = QVBoxLayout()

        self.reset_button = QPushButton("Reset Statistics")
        self.reset_button.clicked.connect(self.reset_statistics)
        control_layout.addWidget(self.reset_button)

        control_group.setLayout(control_layout)
        left_panel.addWidget(control_group)

        left_panel.addStretch()

        main_layout.addLayout(left_panel, 1)

        # Right panel: Graphs and per-joint data
        right_panel = QVBoxLayout()

        # Power plot
        self.power_plot = PowerPlot(self, width=6, height=3)
        right_panel.addWidget(self.power_plot)

        # Per-joint power bars
        joint_group = QGroupBox("Per-Joint Power Consumption")
        joint_layout = QGridLayout()

        self.joint_labels = {}
        self.joint_bars = {}
        self.joint_value_labels = {}

        joints = [
            'shoulder_pitch_joint',
            'shoulder_roll_joint',
            'shoulder_yaw_joint',
            'elbow_pitch_joint',
            'elbow_yaw_joint'
        ]

        joint_display_names = {
            'shoulder_pitch_joint': 'Shoulder Pitch',
            'shoulder_roll_joint': 'Shoulder Roll',
            'shoulder_yaw_joint': 'Shoulder Yaw',
            'elbow_pitch_joint': 'Elbow Pitch',
            'elbow_yaw_joint': 'Elbow Yaw'
        }

        for i, joint in enumerate(joints):
            label = QLabel(joint_display_names.get(joint, joint))
            bar = QProgressBar()
            bar.setMaximum(200)  # 200W max display
            bar.setFormat("%v W")
            value_label = QLabel("0.0 W / 0.0 A")

            joint_layout.addWidget(label, i, 0)
            joint_layout.addWidget(bar, i, 1)
            joint_layout.addWidget(value_label, i, 2)

            self.joint_labels[joint] = label
            self.joint_bars[joint] = bar
            self.joint_value_labels[joint] = value_label

        joint_group.setLayout(joint_layout)
        right_panel.addWidget(joint_group)

        main_layout.addLayout(right_panel, 2)

    def update_power_data(self, result):
        """Update GUI with power calculation result."""
        import time
        current_time = time.time()

        # Update total power and current
        self.total_power_label.setText(f"{result['total_power']:.1f} W")
        self.total_current_label.setText(f"{result['total_current']:.1f} A")
        self.efficiency_label.setText(f"{result['efficiency']*100:.1f} %")

        # Update statistics
        if self.last_update_time > 0:
            dt = current_time - self.last_update_time
            self.total_energy_wh += result['total_power'] * dt / 3600.0
            self.energy_label.setText(f"{self.total_energy_wh:.3f} Wh")

        self.last_update_time = current_time

        if result['total_power'] > self.peak_power_w:
            self.peak_power_w = result['total_power']
            self.peak_power_label.setText(f"{self.peak_power_w:.1f} W")

        self.sample_count += 1
        alpha = 1.0 / min(self.sample_count, 100)
        self.avg_power_w = (1 - alpha) * self.avg_power_w + alpha * result['total_power']
        self.avg_power_label.setText(f"{self.avg_power_w:.1f} W")

        # Update plot
        self.power_plot.update_plot(current_time, result['total_power'], result['total_current'])

        # Update per-joint bars
        for joint, data in result['joints'].items():
            if joint in self.joint_bars:
                power = data['power']
                current = data['current']
                self.joint_bars[joint].setValue(int(power))
                self.joint_value_labels[joint].setText(f"{power:.1f} W / {current:.1f} A")

        # Update battery life estimation
        try:
            battery_ah = float(self.battery_capacity_input.text())
            battery_v = float(self.battery_voltage_input.text())

            if self.avg_power_w > 1.0:
                energy_wh = battery_ah * battery_v * 0.8  # 80% DOD
                runtime_h = energy_wh / self.avg_power_w
                self.battery_life_label.setText(f"{runtime_h:.2f} hours ({runtime_h*60:.0f} min)")
            else:
                self.battery_life_label.setText("-- hours")
        except:
            self.battery_life_label.setText("Invalid input")

    def reset_statistics(self):
        """Reset statistics counters."""
        self.total_energy_wh = 0.0
        self.peak_power_w = 0.0
        self.avg_power_w = 0.0
        self.sample_count = 0
        self.last_update_time = 0

        self.energy_label.setText("0.000 Wh")
        self.peak_power_label.setText("0.0 W")
        self.avg_power_label.setText("0.0 W")

        self.power_plot.time_data.clear()
        self.power_plot.power_data.clear()
        self.power_plot.current_data.clear()
        self.power_plot.start_time = 0


def main(args=None):
    """Main entry point."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create Qt application
    app = QtWidgets.QApplication(sys.argv)

    # Create GUI
    gui = PowerMonitorGUI()
    gui.show()

    # Create ROS node
    ros_node = PowerMonitorROSNode(gui.update_power_data)

    # Timer to spin ROS
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100 Hz

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
