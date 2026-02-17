#!/usr/bin/env python3
"""PyQt5 GUI with per-tool start/stop buttons for the full system, Gazebo, RViz, and rqt_image_view."""

import os
import shlex
import signal
import subprocess
import sys
from functools import partial
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, uic


ROBOT_CONTROL_CMD = 'ros2 launch arm_control control.launch.py'
SIMULATION_WORLD_CMD = 'ros2 launch arm_gazebo headless_sim.launch.py'
GAZEBO_CMD = 'gz sim -g'
GAZEBO_EXTERNAL_CMD = 'ros2 launch arm_gazebo gz_gui.launch.py'
RVIZ_CMD = 'rviz2 -d $(ros2 pkg prefix arm_perception)/share/arm_perception/config/deep_camera.rviz'
MOVEIT_SERVER_CMD = 'ros2 launch arm_moveit_config move_group.launch.py'
MOVEIT_CLIENT_CMD = 'ros2 launch arm_moveit_config moveit_rviz.launch.py'
OCTOMAP_CMD = 'ros2 launch arm_system_bringup moveit_octomap_only.launch.py'
OBJECT_DETECTION_CMD = 'ros2 run arm_perception object_recognition_node.py'
YOLO_TRACKING_CMD = '~/ldr-humanoid-arm-system/yolov8_native_tracking.py'
VISUAL_ODOMETRY_CMD = '~/ldr-humanoid-arm-system/visual_odometry_exact.py'
PERCEPTIION_CMD = 'ros2 launch arm_perception perception.launch.py'


class LauncherWindow(QtWidgets.QMainWindow):
    """GUI that starts/stops each ROS 2 tool inros2 launch arm_system_bringup octomap_server.launch.py dependently in the background."""

    def __init__(self):
        super().__init__()
        self._ui_root = None
        self._branding_label = None
        self._branding_pixmap = None
        self._load_ui()
        self._set_branding_image()

        self.command_line = self.findChild(QtWidgets.QLineEdit, 'line_command')

        self._setup_script = None
        self.tools = {}
        self._world_combo = None
        self._current_world_path = ''

        self._external_server_checkbox = None
        self._external_ip_edit = None
        self._external_user_edit = None
        self._external_password_edit = None
        self._moveit_client_widgets = None

        self._setup_world_selector()
        self._setup_moveit_split_controls()
        self._setup_external_server()
        self._update_launch_command()
        self._register_tool(
            name='system',
            command=ROBOT_CONTROL_CMD,
            start_button='button_system_start',
            stop_button='button_system_stop',
            status_label='label_system_status',
        )
        self._register_tool(
            name='simulation_world',
            command=self._build_launch_command(),
            start_button='button_simulation_world_start',
            stop_button='button_simulation_world_stop',
            status_label='label_simulation_world_status',
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
            name='moveit_client',
            command=MOVEIT_CLIENT_CMD,
            start_button=self._moveit_client_widgets['start_button'],
            stop_button=self._moveit_client_widgets['stop_button'],
            status_label=self._moveit_client_widgets['status_label'],
        )
        self._register_tool(
            name='moveit_server',
            command=MOVEIT_SERVER_CMD,
            start_button='button_moveit_start',
            stop_button='button_moveit_stop',
            status_label='label_moveit_status',
        )
        self._register_tool(
            name='octomap',
            command=OCTOMAP_CMD,
            start_button='button_octomap_start',
            stop_button='button_octomap_stop',
            status_label='label_octomap_status',
        )
        self._register_tool(
            name='object_detection',
            command=OBJECT_DETECTION_CMD,
            start_button='button_object_detection_start',
            stop_button='button_object_detection_stop',
            status_label='label_object_detection_status',
        )
        self._register_tool(
            name='yolo',
            command=YOLO_TRACKING_CMD,
            start_button='button_yolo_start',
            stop_button='button_yolo_stop',
            status_label='label_yolo_status',
        )
        self._register_tool(
            name='vo',
            command=VISUAL_ODOMETRY_CMD,
            start_button='button_vo_start',
            stop_button='button_vo_stop',
            status_label='label_vo_status',
        )
        self._register_tool(
            name='perception',
            command=PERCEPTIION_CMD,
            start_button='button_perception_start',
            stop_button='button_perception_stop',
            status_label='label_perception_status',
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

        image_path = self._resolve_ui_path(Path('images') / 'image.jpg')
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

    def _setup_world_selector(self):
        """Create a combo box that lists available Gazebo world files."""
        group_box = self.findChild(QtWidgets.QGroupBox, 'group_processes')
        grid_layout = group_box.layout() if group_box else None
        if not isinstance(grid_layout, QtWidgets.QGridLayout):
            return

        label = QtWidgets.QLabel('Simulation World', self)
        label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        combo = QtWidgets.QComboBox(self)
        combo.setObjectName('combo_simulation_world')
        combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        combo.currentIndexChanged.connect(self._on_world_selection_changed)

        row = grid_layout.rowCount()
        grid_layout.addWidget(label, row, 0)
        grid_layout.addWidget(combo, row, 1, 1, 3)

        self._world_combo = combo
        self._populate_world_selector()

    def _setup_external_server(self):
        """Add an 'External Server' checkbox with IP, user and password fields."""
        group_box = self.findChild(QtWidgets.QGroupBox, 'group_processes')
        grid_layout = group_box.layout() if group_box else None
        if not isinstance(grid_layout, QtWidgets.QGridLayout):
            return

        checkbox = QtWidgets.QCheckBox('External Server', self)
        checkbox.setObjectName('checkbox_external_server')
        checkbox.setToolTip('Connect to a headless Gazebo server running on a remote machine (e.g. Jetson)')

        ip_edit = QtWidgets.QLineEdit(self)
        ip_edit.setObjectName('edit_external_ip')
        ip_edit.setPlaceholderText('IP (e.g. 192.168.1.50)')
        ip_edit.setVisible(False)

        user_edit = QtWidgets.QLineEdit(self)
        user_edit.setObjectName('edit_external_user')
        user_edit.setPlaceholderText('User')
        user_edit.setText('nano')
        user_edit.setMaximumWidth(100)
        user_edit.setVisible(False)

        password_edit = QtWidgets.QLineEdit(self)
        password_edit.setObjectName('edit_external_password')
        password_edit.setPlaceholderText('Password')
        password_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        password_edit.setMaximumWidth(120)
        password_edit.setVisible(False)

        row = grid_layout.rowCount()
        grid_layout.addWidget(checkbox, row, 0)
        grid_layout.addWidget(ip_edit, row, 1)
        grid_layout.addWidget(user_edit, row, 2)
        grid_layout.addWidget(password_edit, row, 3)

        self._external_server_checkbox = checkbox
        self._external_ip_edit = ip_edit
        self._external_user_edit = user_edit
        self._external_password_edit = password_edit

        checkbox.toggled.connect(self._on_external_server_toggled)

    def _setup_moveit_split_controls(self):
        """Reuse existing MoveIt controls for server and add a client row."""
        group_box = self.findChild(QtWidgets.QGroupBox, 'group_processes')
        grid_layout = group_box.layout() if group_box else None
        if not isinstance(grid_layout, QtWidgets.QGridLayout):
            return

        moveit_server_label = self.findChild(QtWidgets.QLabel, 'label_moveit_name')
        if moveit_server_label:
            moveit_server_label.setText('MoveIt Server')

        label = QtWidgets.QLabel('MoveIt Client', self)

        start_btn = QtWidgets.QPushButton('Start', self)
        start_btn.setObjectName('button_moveit_client_start')
        stop_btn = QtWidgets.QPushButton('Stop', self)
        stop_btn.setObjectName('button_moveit_client_stop')
        status_lbl = QtWidgets.QLabel('Idle', self)
        status_lbl.setObjectName('label_moveit_client_status')

        row = self._insert_row_below_widget(grid_layout, moveit_server_label)
        grid_layout.addWidget(label, row, 0)
        grid_layout.addWidget(start_btn, row, 1)
        grid_layout.addWidget(stop_btn, row, 2)
        grid_layout.addWidget(status_lbl, row, 3)

        self._moveit_client_widgets = {
            'start_button': start_btn,
            'stop_button': stop_btn,
            'status_label': status_lbl,
        }

    @staticmethod
    def _insert_row_below_widget(grid_layout, widget):
        """Insert a new row below widget's row by shifting lower rows down."""
        if widget is None:
            return grid_layout.rowCount()

        index = grid_layout.indexOf(widget)
        if index < 0:
            return grid_layout.rowCount()

        server_row, _, _, _ = grid_layout.getItemPosition(index)
        insert_row = server_row + 1

        entries = []
        for i in range(grid_layout.count()):
            item = grid_layout.itemAt(i)
            child_widget = item.widget()
            if child_widget is None:
                continue
            row, col, row_span, col_span = grid_layout.getItemPosition(i)
            entries.append((child_widget, row, col, row_span, col_span))

        for child_widget, _, _, _, _ in entries:
            grid_layout.removeWidget(child_widget)

        for child_widget, row, col, row_span, col_span in entries:
            new_row = row + 1 if row >= insert_row else row
            grid_layout.addWidget(child_widget, new_row, col, row_span, col_span)

        return insert_row

    def _on_external_server_toggled(self, checked):
        """Show/hide connection fields and swap Gazebo command."""
        self._external_ip_edit.setVisible(checked)
        self._external_user_edit.setVisible(checked)
        self._external_password_edit.setVisible(checked)

        # Swap Gazebo command
        self._update_gazebo_external_command()
        self._update_tool_buttons('moveit_server')
        self._update_tool_buttons('moveit_client')
        moveit_server_tool = self.tools.get('moveit_server')
        if moveit_server_tool and checked and not self._is_running(moveit_server_tool):
            self._set_tool_status(moveit_server_tool, 'Disabled (external server)')

        # Re-update gazebo command when any field changes
        fields = [self._external_ip_edit, self._external_user_edit, self._external_password_edit]
        if checked:
            for field in fields:
                field.textChanged.connect(self._on_external_field_changed)
        else:
            for field in fields:
                try:
                    field.textChanged.disconnect(self._on_external_field_changed)
                except TypeError:
                    pass

    def _on_external_field_changed(self):
        """Update the Gazebo command when any external server field changes."""
        self._update_gazebo_external_command()

    def _update_gazebo_external_command(self):
        """Build the Gazebo command based on external server fields."""
        gazebo_tool = self.tools.get('gazebo')
        if not gazebo_tool:
            return

        if not (self._external_server_checkbox and self._external_server_checkbox.isChecked()):
            gazebo_tool['command'] = GAZEBO_CMD
            return

        ip = self._external_ip_edit.text().strip()
        user = self._external_user_edit.text().strip()
        password = self._external_password_edit.text()

        cmd = GAZEBO_EXTERNAL_CMD
        if ip:
            cmd += f' jetson_ip:={ip}'
        if user:
            cmd += f' jetson_user:={user}'
        if password:
            cmd += f' jetson_password:={password}'
        gazebo_tool['command'] = cmd

    def _populate_world_selector(self):
        """Fill the combo box with *.sdf files from arm_gazebo/worlds."""
        if self._world_combo is None:
            return

        worlds = self._discover_world_files()
        combo = self._world_combo
        combo.blockSignals(True)
        combo.clear()

        if not worlds:
            combo.addItem('No .sdf worlds found', '')
            combo.setEnabled(False)
            self._current_world_path = ''
        else:
            combo.setEnabled(True)
            for sdf_path in worlds:
                combo.addItem(sdf_path.name, str(sdf_path))
            combo.setCurrentIndex(0)
            self._current_world_path = str(worlds[0])

        combo.blockSignals(False)
        self._update_launch_command()

    def _discover_world_files(self):
        """Return a sorted list of available world files."""
        directories = []
        try:
            from ament_index_python.packages import get_package_share_directory

            directories.append(Path(get_package_share_directory('arm_gazebo')) / 'worlds')
        except Exception:
            pass

        repo_worlds = self._locate_repo_worlds_dir()
        if repo_worlds:
            directories.append(repo_worlds)

        worlds = []
        seen = set()
        for directory in directories:
            if not directory or not directory.exists():
                continue
            for sdf in sorted(directory.glob('*.sdf')):
                resolved = str(sdf.resolve())
                if resolved in seen:
                    continue
                seen.add(resolved)
                worlds.append(Path(resolved))

        return worlds

    @staticmethod
    def _locate_repo_worlds_dir():
        """Search upwards for the workspace source tree and worlds folder."""
        current = Path(__file__).resolve()
        for parent in [current, *current.parents]:
            candidate = parent / 'src' / 'simulation' / 'arm_gazebo' / 'worlds'
            if candidate.exists():
                return candidate
        return None

    def _on_world_selection_changed(self, index):
        """Store the newly selected world path and refresh the launch command."""
        if self._world_combo is None or index < 0:
            return
        world_path = self._world_combo.itemData(index) or ''
        self._current_world_path = world_path
        self._update_launch_command()

    def _build_launch_command(self):
        """Produce the ros2 launch command with the selected world argument."""
        command = SIMULATION_WORLD_CMD
        if self._current_world_path:
            command = f"{command} simulation_world:={shlex.quote(self._current_world_path)}"
        return command

    def _update_launch_command(self):
        """Synchronize the displayed launch command and tool entry."""
        command = self._build_launch_command()
        if self.command_line:
            self.command_line.setText(command)
        simulation_tool = self.tools.get('simulation_world')
        if simulation_tool:
            simulation_tool['command'] = command

    def _register_tool(self, name, command, start_button, stop_button, status_label):
        start_btn = (
            self._require_widget(QtWidgets.QPushButton, start_button)
            if isinstance(start_button, str)
            else start_button
        )
        stop_btn = (
            self._require_widget(QtWidgets.QPushButton, stop_button)
            if isinstance(stop_button, str)
            else stop_button
        )
        status_lbl = (
            self._require_widget(QtWidgets.QLabel, status_label)
            if isinstance(status_label, str)
            else status_label
        )

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

        if name == 'moveit_server':
            if self._external_server_checkbox and self._external_server_checkbox.isChecked():
                self._set_tool_status(tool, 'Disabled (external server)')
                self._update_tool_buttons(name)
                return
        if name == 'moveit_client':
            external = bool(self._external_server_checkbox and self._external_server_checkbox.isChecked())
            moveit_server_tool = self.tools.get('moveit_server')
            if (not external) and (not moveit_server_tool or not self._is_running(moveit_server_tool)):
                self._set_tool_status(tool, 'Start MoveIt Server first.')
                self._update_tool_buttons(name)
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
        if name == 'moveit_server':
            self._update_tool_buttons('moveit_client')

    def stop_tool(self, name):
        tool = self.tools[name]
        if self._terminate_process(tool):
            self._set_tool_status(tool, 'Stop signal sent.')
        else:
            self._set_tool_status(tool, 'Not running.')
        self._update_tool_buttons(name)
        if name == 'moveit_server':
            moveit_client_tool = self.tools.get('moveit_client')
            if moveit_client_tool and not self._is_running(moveit_client_tool):
                self._set_tool_status(moveit_client_tool, 'Start MoveIt Server first.')
            self._update_tool_buttons('moveit_client')

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
                if name == 'moveit_server':
                    moveit_client_tool = self.tools.get('moveit_client')
                    if moveit_client_tool and not self._is_running(moveit_client_tool):
                        self._set_tool_status(moveit_client_tool, 'Start MoveIt Server first.')
                    self._update_tool_buttons('moveit_client')

    @staticmethod
    def _is_running(tool):
        return bool(tool['process'] and tool['process'].poll() is None)

    def _update_tool_buttons(self, name):
        tool = self.tools[name]
        running = self._is_running(tool)
        tool['start_button'].setEnabled(not running)
        tool['stop_button'].setEnabled(running)
        if name == 'moveit_server':
            external = bool(self._external_server_checkbox and self._external_server_checkbox.isChecked())
            tool['start_button'].setEnabled((not running) and (not external))
        if name == 'moveit_client':
            external = bool(self._external_server_checkbox and self._external_server_checkbox.isChecked())
            moveit_server_tool = self.tools.get('moveit_server')
            server_running = bool(moveit_server_tool and self._is_running(moveit_server_tool))
            tool['start_button'].setEnabled((not running) and (external or server_running))

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
