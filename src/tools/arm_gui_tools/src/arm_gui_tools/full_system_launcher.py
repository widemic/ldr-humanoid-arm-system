#!/usr/bin/env python3
"""PyQt5 GUI with per-tool start/stop buttons for the full system, Gazebo, RViz, and rqt_image_view."""

import os
import signal
import subprocess
import sys
from functools import partial
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, uic


LAUNCH_CMD = 'ros2 launch arm_system_bringup full_system.launch.py'
IMAGE_VIEW_CMD = 'ros2 run image_tools showimage --ros-args -r image:=/camera/color/image_raw'
GAZEBO_CMD = 'gz sim -g'
RVIZ_CMD = 'rviz2'
MOVEIT_CMD = 'ros2 launch arm_moveit_config demo.launch.py'
OBJECT_DETECTION_CMD = 'ros2 run arm_perception object_recognition_node.py'


class LauncherWindow(QtWidgets.QMainWindow):
    """GUI that starts/stops each ROS 2 tool independently in the background."""

    def __init__(self):
        super().__init__()
        self._ui_root = None
        self._branding_label = None
        self._branding_pixmap = None
        self._load_ui()
        self._set_branding_image()

        self.command_line = self.findChild(QtWidgets.QLineEdit, 'line_command')
        if self.command_line:
            self.command_line.setText(LAUNCH_CMD)

        self._setup_script = None
        self.tools = {}
        self._register_tool(
            name='system',
            command=LAUNCH_CMD,
            start_button='button_system_start',
            stop_button='button_system_stop',
            status_label='label_system_status',
        )
        self._register_tool(
            name='rqt',
            command=IMAGE_VIEW_CMD,
            start_button='button_rqt_start',
            stop_button='button_rqt_stop',
            status_label='label_rqt_status',
        )
        self._register_tool(
            name='gazebo',
            command=GAZEBO_CMD,
            start_button='button_gazebo_start',
            stop_button='button_gazebo_stop',
            status_label='label_gazebo_status',
        )
        self._register_tool(
            name='rviz',
            command=RVIZ_CMD,
            start_button='button_rviz_start',
            stop_button='button_rviz_stop',
            status_label='label_rviz_status',
        )
        self._register_tool(
            name='moveit',
            command=MOVEIT_CMD,
            start_button='button_moveit_start',
            stop_button='button_moveit_stop',
            status_label='label_moveit_status',
        )
        self._register_tool(
            name='object_detection',
            command=OBJECT_DETECTION_CMD,
            start_button='button_object_detection_start',
            stop_button='button_object_detection_stop',
            status_label='label_object_detection_status',
        )

        self.monitor_timer = QtCore.QTimer(self)
        self.monitor_timer.timeout.connect(self._cleanup_finished_processes)
        self.monitor_timer.start(1000)

    def _load_ui(self):
        """Load the Qt Designer file either from install or source tree."""
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = Path(pkg_share) / 'ui' / 'full_system_launcher.ui'
        except Exception:
            ui_file = Path(__file__).resolve().parent.parent.parent / 'ui' / 'full_system_launcher.ui'

        uic.loadUi(str(ui_file), self)
        self._ui_root = ui_file.parent

    def _resolve_ui_path(self, relative):
        """Resolve a path inside the UI directory regardless of install/source context."""
        relative = Path(relative)
        if self._ui_root:
            candidate = self._ui_root / relative
            if candidate.exists():
                return candidate

        fallback_root = Path(__file__).resolve().parent.parent.parent / 'ui'
        candidate = fallback_root / relative
        if candidate.exists():
            return candidate
        return None

    def _set_branding_image(self):
        """Attach the Love Death + Robots artwork to the placeholder label."""
        label = self.findChild(QtWidgets.QLabel, 'label_branding')
        if not label:
            return
        self._branding_label = label

        image_path = self._resolve_ui_path(Path('images') / 'love_death_robots.png')
        if image_path:
            pixmap = QtGui.QPixmap(str(image_path))
            if not pixmap.isNull():
                self._branding_pixmap = pixmap
                label.installEventFilter(self)
                label.setAlignment(QtCore.Qt.AlignCenter)
                self._update_branding_pixmap()
                return

        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setText('LOVE DEATH + ROBOTS')

    def _update_branding_pixmap(self):
        """Scale the branding art to fill available space while preserving aspect ratio."""
        if not (self._branding_label and self._branding_pixmap):
            return

        target_size = self._branding_label.size()
        if target_size.width() <= 0 or target_size.height() <= 0:
            return

        scaled = self._branding_pixmap.scaled(
            target_size,
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self._branding_label.setPixmap(scaled)

    def eventFilter(self, watched, event):
        if (
            watched is self._branding_label
            and event.type() == QtCore.QEvent.Resize
            and self._branding_pixmap is not None
        ):
            self._update_branding_pixmap()
        return super().eventFilter(watched, event)

    def _register_tool(self, name, command, start_button, stop_button, status_label):
        start_btn = self._require_widget(QtWidgets.QPushButton, start_button)
        stop_btn = self._require_widget(QtWidgets.QPushButton, stop_button)
        status_lbl = self._require_widget(QtWidgets.QLabel, status_label)

        tool = {
            'command': command,
            'start_button': start_btn,
            'stop_button': stop_btn,
            'status_label': status_lbl,
            'process': None,
        }
        self.tools[name] = tool

        start_btn.clicked.connect(partial(self.start_tool, name))
        stop_btn.clicked.connect(partial(self.stop_tool, name))
        self._update_tool_buttons(name)

    def _require_widget(self, widget_cls, object_name):
        widget = self.findChild(widget_cls, object_name)
        if widget is None:
            raise RuntimeError(f'{object_name} not found in UI file')
        return widget

    def start_tool(self, name):
        tool = self.tools[name]
        self._cleanup_finished_processes()

        if self._is_running(tool):
            self._set_tool_status(tool, 'Already running.')
            return

        try:
            tool['process'] = self._start_process(tool['command'])
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                f'{name} failed',
                f'Failed to start command:\n{exc}'
            )
            self._set_tool_status(tool, 'Failed to start.')
            return

        self._set_tool_status(tool, f'Running (pid {tool["process"].pid}).')
        self._update_tool_buttons(name)

    def stop_tool(self, name):
        tool = self.tools[name]
        if self._terminate_process(tool):
            self._set_tool_status(tool, 'Stop signal sent.')
        else:
            self._set_tool_status(tool, 'Not running.')
        self._update_tool_buttons(name)

    def _start_process(self, command):
        """Launch a ROS command in the background using bash."""
        wrapped = self._wrap_with_setup(command)
        return subprocess.Popen(
            ['bash', '-lc', wrapped],
            preexec_fn=os.setsid
        )

    def _wrap_with_setup(self, command):
        setup_script = self._get_setup_script()
        if setup_script:
            return f'source "{setup_script}" && {command}'
        return command

    def _get_setup_script(self):
        if self._setup_script is None:
            self._setup_script = self._locate_setup_script()
        return self._setup_script

    @staticmethod
    def _locate_setup_script():
        """Attempt to locate install/setup.bash relative to this file."""
        current = Path(__file__).resolve()
        for parent in [current, *current.parents]:
            candidate = parent / 'install' / 'setup.bash'
            if candidate.exists():
                return candidate
        # Fallback for development (assume environment already sourced)
        return None

    def _terminate_process(self, tool):
        """Send SIGINT (then SIGTERM) to the stored process."""
        process = tool['process']
        if not process:
            return False

        if process.poll() is not None:
            tool['process'] = None
            return False

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        finally:
            tool['process'] = None

        return True

    def _cleanup_finished_processes(self):
        """Reset handles when processes exit on their own."""
        for name, tool in self.tools.items():
            if tool['process'] and tool['process'].poll() is not None:
                tool['process'] = None
                self._set_tool_status(tool, 'Exited.')
                self._update_tool_buttons(name)

    @staticmethod
    def _is_running(tool):
        return bool(tool['process'] and tool['process'].poll() is None)

    def _update_tool_buttons(self, name):
        tool = self.tools[name]
        running = self._is_running(tool)
        tool['start_button'].setEnabled(not running)
        tool['stop_button'].setEnabled(running)

    @staticmethod
    def _set_tool_status(tool, text):
        tool['status_label'].setText(text)


def main():
    """Entry point for manual testing."""
    app = QtWidgets.QApplication(sys.argv)
    window = LauncherWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
