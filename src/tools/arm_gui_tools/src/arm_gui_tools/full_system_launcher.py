#!/usr/bin/env python3
"""PyQt5 GUI with per-tool start/stop/restart buttons for the full system, Gazebo, RViz, and related tools.
Also supports per-tool "Skip Stop All" checkbox to exclude selected tools from Stop All.
"""

import os
import shlex
import signal
import subprocess
import sys
import re
import json
from functools import partial
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, uic


FULL_SYSTEM_BASE_CMD = 'ros2 launch arm_system_bringup full_system.launch.py'
IMAGE_VIEW_CMD = 'ros2 run image_tools showimage --ros-args -r image:=/camera/color/image_raw'
GAZEBO_CMD = 'gz sim -g'
RVIZ_CMD = 'rviz2 -d $(ros2 pkg prefix arm_perception)/share/arm_perception/config/deep_camera.rviz'
MOVEIT_CMD = 'ros2 launch arm_moveit_config demo.launch.py'
OCTOMAP_CMD = 'ros2 launch arm_system_bringup moveit_octomap_only.launch.py'
OBJECT_DETECTION_CMD = 'ros2 run arm_perception object_recognition_node.py'
YOLO_TRACKING_CMD = '/home/andrei/ros2_ws/ldr-humanoid-arm-system/yolov8_native_tracking.py'
VISUAL_ODOMETRY_CMD = '/home/andrei/ros2_ws/ldr-humanoid-arm-system/visual_odometry_exact.py'
PERCEPTION_CMD = 'ros2 launch arm_perception perception.launch.py'
CONTROLLERS_CMD = 'ros2 control list_controllers'


class LauncherWindow(QtWidgets.QMainWindow):
    """GUI that starts/stops/restarts each ROS 2 tool in the background."""

    def __init__(self):
        super().__init__()

        self._ui_root = None
        self._branding_label = None
        self._branding_pixmap = None
        self._setup_script = None

        self.tools = {}
        self._world_combo = None
        self._current_world_path = ''

        # ROS readiness checker state
        self._system_ready = False
        self._ros_check_process = None

        # Controller listing state
        self.controllers_list = None
        self._controllers_process = None
        self._controllers_timer = None

        # Stop-all state
        self._reset_timer = None
        self._reset_countdown = 0.0
        self._reset_running_tools = []
        self._reset_still_running = []

        self._load_ui()
        self._set_branding_image()

        # Optional command-line display (may be None if not in UI)
        self.command_line = self.findChild(QtWidgets.QLineEdit, 'line_command')


        # Tools/config loading (decouple GUI wiring from commands)
        self._config = self._load_tools_config()

        fs_cfg = self._config.get('full_system', {})
        self._full_system_base_cmd = fs_cfg.get('base_command', FULL_SYSTEM_BASE_CMD)
        self._full_system_world_arg_key = fs_cfg.get('world_arg_key', 'simulation_world')

        self._setup_world_selector()
        self._update_launch_command()

        # ------------------------------------------------------------------
        # Register tools (buttons + status labels + (optional) restart + skip must exist in UI)
        # ------------------------------------------------------------------
        for t in self._config.get('tools', []):
            name = t['name']
            cmd = t.get('command', '')

            # Special-case: full system launch command is built dynamically based on world selection
            if cmd == '__FULL_SYSTEM__':
                cmd = self._build_launch_command()

            self._register_tool(
                name=name,
                command=cmd,
                start_button=t['start_button'],
                stop_button=t['stop_button'],
                restart_button=t.get('restart_button'),
                status_label=t['status_label'],
                skip_checkbox=t.get('skip_checkbox'),
            )

        # Wire Stop All button (objectName remains button_reset_all)
        stop_all_btn = self.findChild(QtWidgets.QPushButton, 'button_reset_all')
        if stop_all_btn is not None:
            stop_all_btn.clicked.connect(self._reset_all_processes)

        # Controllers list widget
        self.controllers_list = self.findChild(QtWidgets.QListWidget, 'list_controllers')

        # Asynchronous ROS readiness checker (non-blocking) every 2s
        self._ros_check_timer = QtCore.QTimer(self)
        self._ros_check_timer.timeout.connect(self._start_async_ros_check)
        self._ros_check_timer.start(2000)

        # Periodic controller status monitor (every 3s)
        self._controllers_timer = QtCore.QTimer(self)
        self._controllers_timer.timeout.connect(self._start_controllers_query)
        self._controllers_timer.start(3000)

        # Initial button state
        self._update_all_buttons()

    # ------------------------------------------------------------------
    # UI loading / layout helpers
    # ------------------------------------------------------------------

    def _load_ui(self):
        """Load the Qt Designer .ui file from install or source tree."""
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = Path(pkg_share) / 'ui' / 'full_system_launcher.ui'
        except Exception:
            ui_file = Path(__file__).resolve().parent.parent.parent / 'ui' / 'full_system_launcher.ui'

        uic.loadUi(str(ui_file), self)
        self._ui_root = ui_file.parent


    def _load_tools_config(self) -> dict:
        """Load launcher_tools.json either next to this script or from the package share directory."""
        candidates = []

        # 1) Next to this script (development / run-from-source)
        candidates.append(Path(__file__).resolve().parent / 'launcher_tools.json')

        # 2) Installed package share (arm_gui_tools/config/launcher_tools.json)
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = Path(get_package_share_directory('arm_gui_tools'))
            candidates.append(pkg_share / 'config' / 'launcher_tools.json')
        except Exception:
            pass

        for p in candidates:
            if p.exists():
                with p.open('r', encoding='utf-8') as f:
                    return json.load(f)

        raise FileNotFoundError(
            'launcher_tools.json not found. Looked in:\n' + '\n'.join(str(p) for p in candidates)
        )

    def _resolve_ui_path(self, relative: Path) -> Path | None:
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
        """Attach artwork to the placeholder label (label_branding)."""
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

    # ------------------------------------------------------------------
    # World selector / full-system command construction
    # ------------------------------------------------------------------

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
        command = getattr(self, '_full_system_base_cmd', FULL_SYSTEM_BASE_CMD)
        if self._current_world_path:
            key = getattr(self, '_full_system_world_arg_key', 'simulation_world')
            command = f"{command} {key}:={shlex.quote(self._current_world_path)}"
        return command

    def _update_launch_command(self):
        """Synchronize the displayed launch command and tool entry."""
        command = self._build_launch_command()
        if self.command_line is not None:
            self.command_line.setText(command)
        system_tool = self.tools.get('system')
        if system_tool:
            system_tool['command'] = command

    # ------------------------------------------------------------------
    # Tool registration / process management
    # ------------------------------------------------------------------

    def _register_tool(
        self,
        name,
        command,
        start_button,
        stop_button,
        status_label,
        restart_button=None,
        skip_checkbox=None,
    ):
        start_btn = self._require_widget(QtWidgets.QPushButton, start_button)
        stop_btn = self._require_widget(QtWidgets.QPushButton, stop_button)
        status_lbl = self._require_widget(QtWidgets.QLabel, status_label)

        restart_btn = None
        if restart_button is not None:
            restart_btn = self._require_widget(QtWidgets.QPushButton, restart_button)

        skip_cb = None
        if skip_checkbox is not None:
            skip_cb = self._require_widget(QtWidgets.QCheckBox, skip_checkbox)

        tool = {
            'command': command,
            'start_button': start_btn,
            'stop_button': stop_btn,
            'restart_button': restart_btn,
            'skip_checkbox': skip_cb,
            'status_label': status_lbl,
            'process': None,
        }
        self.tools[name] = tool

        start_btn.clicked.connect(partial(self.start_tool, name))
        stop_btn.clicked.connect(partial(self.stop_tool, name))
        if restart_btn is not None:
            restart_btn.clicked.connect(partial(self.restart_tool, name))

        self._update_tool_buttons(name)

    def _require_widget(self, widget_cls, object_name):
        widget = self.findChild(widget_cls, object_name)
        if widget is None:
            raise RuntimeError(f'{object_name} not found in UI file')
        return widget

    def start_tool(self, name):
        tool = self.tools[name]
        self._cleanup_finished_processes()

        # Block starting other tools if system is not configured or running
        if name != 'system' and not self._ensure_system_running():
            return

        if self._is_running(tool):
            self._set_tool_status(tool, 'Already running.')
            return

        try:
            tool['process'] = self._start_process(tool['command'])
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                f'{name} failed',
                f'Failed to start command:\n{exc}',
            )
            self._set_tool_status(tool, 'Failed to start.')
            return

        self._set_tool_status(tool, f'Running (pid {tool["process"].pid}).')
        self._update_tool_buttons(name)

        # If system just started, schedule an initial controller list update after 2 seconds
        if name == 'system':
            QtCore.QTimer.singleShot(2000, self._start_controllers_query)

    def stop_tool(self, name):
        tool = self.tools[name]
        if self._terminate_process(tool):
            self._set_tool_status(tool, 'Stop signal sent.')
        else:
            self._set_tool_status(tool, 'Not running.')
        self._update_tool_buttons(name)

    def restart_tool(self, name):
        """Stop (if running) then start again."""
        tool = self.tools[name]
        self._cleanup_finished_processes()

        # For non-system tools, require full system ready
        if name != 'system' and not self._ensure_system_running():
            return

        # Stop if running
        if self._is_running(tool):
            self._set_tool_status(tool, 'Restarting...')
            self._terminate_process(tool)

        # Start
        try:
            tool['process'] = self._start_process(tool['command'])
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                f'{name} restart failed',
                f'Failed to restart command:\n{exc}',
            )
            self._set_tool_status(tool, 'Failed to restart.')
            tool['process'] = None
            self._update_tool_buttons(name)
            return

        self._set_tool_status(tool, f'Running (pid {tool["process"].pid}).')
        self._update_tool_buttons(name)

        if name == 'system':
            QtCore.QTimer.singleShot(2000, self._start_controllers_query)

    def _start_process(self, command):
        """Launch a ROS command in the background using bash."""
        wrapped = self._wrap_with_setup(command)
        return subprocess.Popen(
            ['bash', '-lc', wrapped],
            preexec_fn=os.setsid,
        )

    def _wrap_with_setup(self, command: str) -> str:
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
        """Terminate the stored process, escalate to SIGKILL if needed."""
        process = tool.get('process')
        if not process:
            return False

        if process.poll() is not None:
            tool['process'] = None
            return False

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
        finally:
            tool['process'] = None

        return True

    def _cleanup_finished_processes(self):
        """Reset handles when processes exit on their own and refresh UI state."""
        for name, tool in self.tools.items():
            if tool['process'] and tool['process'].poll() is not None:
                tool['process'] = None
                self._set_tool_status(tool, 'Exited.')

        # Refresh button states after cleanup
        self._update_all_buttons()

    # ------------------------------------------------------------------
    # Stop-all logic (GUI-launched + external ROS/Gazebo/MoveIt processes)
    # ------------------------------------------------------------------

    def _reset_all_processes(self):
        """Stop all GUI-managed tools (except those checked 'Skip Stop All') then kill external ROS/Gazebo/MoveIt."""
        self._cleanup_finished_processes()

        running_tools = []
        for name, tool in self.tools.items():
            # Respect per-tool skip checkbox
            skip_cb = tool.get('skip_checkbox')
            if skip_cb is not None and skip_cb.isChecked():
                continue

            if self._is_running(tool):
                running_tools.append((name, tool))
                try:
                    os.killpg(os.getpgid(tool['process'].pid), signal.SIGINT)
                    self._set_tool_status(tool, 'Stopping...')
                except Exception:
                    pass

        self._reset_running_tools = running_tools
        self._reset_still_running = running_tools.copy()

        if not running_tools:
            self._kill_external_ros_processes()
            QtWidgets.QMessageBox.information(
                self,
                'Stop All',
                'No GUI processes running (or all were skipped). ROS-related processes have been cleaned up.',
            )
            return

        # Start countdown timer to wait for graceful exit
        self._reset_countdown = 5.0  # seconds
        if self._reset_timer is None:
            self._reset_timer = QtCore.QTimer(self)
            self._reset_timer.timeout.connect(self._reset_countdown_tick)
        if not self._reset_timer.isActive():
            self._reset_timer.start(500)  # 500 ms

        self._reset_countdown_tick()

    def _reset_countdown_tick(self):
        """Called periodically to check process status and complete stop-all."""
        still_running = []
        for name, tool in self._reset_still_running:
            if tool['process'] and tool['process'].poll() is None:
                still_running.append((name, tool))
        self._reset_still_running = still_running

        # All GUI-owned processes have exited
        if not self._reset_still_running:
            if self._reset_timer and self._reset_timer.isActive():
                self._reset_timer.stop()

            self._kill_external_ros_processes()
            self._cleanup_finished_processes()
            return

        # Countdown reached 0: force-kill remaining GUI-owned processes
        if self._reset_countdown <= 0:
            if self._reset_timer and self._reset_timer.isActive():
                self._reset_timer.stop()

            for name, tool in self._reset_still_running:
                if tool['process'] and tool['process'].poll() is None:
                    try:
                        os.killpg(os.getpgid(tool['process'].pid), signal.SIGKILL)
                        self._set_tool_status(tool, 'Force-killed.')
                    except Exception:
                        pass
                    finally:
                        tool['process'] = None

            self._kill_external_ros_processes()
            self._cleanup_finished_processes()
            return

        self._reset_countdown -= 0.5

    def _kill_external_ros_processes(self):
        """
        Kill external ROS/Gazebo/MoveIt-related processes, BUT respect per-tool skip checkboxes.
        If a tool is checked "Skip Stop All", we will not run pkill patterns that would match it.
        """

        def skipped(tool_name: str) -> bool:
            t = self.tools.get(tool_name)
            if not t:
                return False
            cb = t.get('skip_checkbox')
            return bool(cb is not None and cb.isChecked())

        # Start with the full list
        patterns = [
            'ros2 ',           # generic ros2 CLI and launch processes
            'gz ',             # Gazebo / gz sim
            'gazebo',          # older Gazebo processes
            'rviz2',           # RViz2
            'move_group',      # MoveIt main process
            'object_recognition_node.py',
            'yolov8_native_tracking.py',
            'visual_odometry_exact.py',
            'image_tools showimage',
            'perception.launch.py',
        ]

        # Remove patterns that correspond to tools the user wants to keep alive
        if skipped('gazebo'):
            patterns = [p for p in patterns if p not in ('gz ', 'gazebo')]

        if skipped('rviz'):
            patterns = [p for p in patterns if p != 'rviz2']

        if skipped('moveit'):
            patterns = [p for p in patterns if p != 'move_group']

        if skipped('object_detection'):
            patterns = [p for p in patterns if p != 'object_recognition_node.py']

        if skipped('yolo'):
            patterns = [p for p in patterns if p != 'yolov8_native_tracking.py']

        if skipped('vo'):
            patterns = [p for p in patterns if p != 'visual_odometry_exact.py']

        if skipped('rqt'):
            patterns = [p for p in patterns if p != 'image_tools showimage']

        if skipped('perception'):
            patterns = [p for p in patterns if p != 'perception.launch.py']

        # Important: if the "system" is skipped, NEVER do broad kills like "ros2 "
        if skipped('system'):
            patterns = [p for p in patterns if p != 'ros2 ']

        # Execute pkill for remaining patterns
        for pat in patterns:
            try:
                subprocess.run(['pkill', '-f', pat], check=False)
            except Exception:
                pass
    # ------------------------------------------------------------------
    # ROS readiness checker
    # ------------------------------------------------------------------

    def _start_async_ros_check(self):
        """Start an asynchronous QProcess to check ROS readiness (non-blocking)."""
        if (
            self._ros_check_process is not None
            and self._ros_check_process.state() == QtCore.QProcess.Running
        ):
            return

        setup = self._get_setup_script()
        if setup:
            cmd = f'source "{setup}" && ros2 node list'
        else:
            cmd = 'ros2 node list'

        proc = QtCore.QProcess(self)
        proc.setProgram('bash')
        proc.setArguments(['-lc', cmd])
        proc.finished.connect(self._on_ros_check_finished)
        self._ros_check_process = proc
        proc.start()

    def _on_ros_check_finished(self, exit_code, exit_status):
        """Handle ROS readiness check completion."""
        ready = False
        try:
            if self._ros_check_process is not None:
                stdout = (
                    self._ros_check_process.readAllStandardOutput()
                    .data()
                    .decode('utf-8', errors='ignore')
                )
                ready = (exit_code == 0 and len(stdout.strip()) > 0)
        except Exception:
            ready = False
        finally:
            if self._ros_check_process is not None:
                self._ros_check_process.deleteLater()
            self._ros_check_process = None

        if ready != self._system_ready:
            self._system_ready = ready
            self._update_all_buttons()

    def _is_ros_system_ready(self):
        """Return cached ROS system readiness (updated asynchronously every 2 seconds)."""
        return bool(self._system_ready)

    # ------------------------------------------------------------------
    # Controller listing (ros2 control list_controllers -> QListWidget)
    # ------------------------------------------------------------------

    def _start_controllers_query(self):
        """Kick off asynchronous 'ros2 control list_controllers' and update list_controllers."""
        if self.controllers_list is None:
            return

        if (
            self._controllers_process is not None
            and self._controllers_process.state() == QtCore.QProcess.Running
        ):
            return

        setup = self._get_setup_script()
        if setup:
            cmd = f'source "{setup}" && {CONTROLLERS_CMD}'
        else:
            cmd = CONTROLLERS_CMD

        proc = QtCore.QProcess(self)
        proc.setProgram('bash')
        proc.setArguments(['-lc', cmd])
        proc.finished.connect(self._on_controllers_process_finished)
        self._controllers_process = proc
        proc.start()

    def _on_controllers_process_finished(self, exit_code, exit_status):
        """Populate list_controllers with the output of ros2 control list_controllers."""
        proc = self._controllers_process
        self._controllers_process = None

        if self.controllers_list is None or proc is None:
            if proc is not None:
                proc.deleteLater()
            return

        try:
            stdout = proc.readAllStandardOutput().data().decode('utf-8', errors='ignore') or ''
            stderr = proc.readAllStandardError().data().decode('utf-8', errors='ignore') or ''
        except Exception:
            stdout = ''
            stderr = ''
        finally:
            proc.deleteLater()

        self.controllers_list.clear()

        lines = [l.strip() for l in stdout.splitlines() if l.strip()]
        if not lines:
            if stderr.strip():
                item = QtWidgets.QListWidgetItem('Error: ' + stderr.splitlines()[0])
                self.controllers_list.addItem(item)
            else:
                item = QtWidgets.QListWidgetItem('No controllers found')
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsSelectable)
                self.controllers_list.addItem(item)
            return

        ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')

        for raw_line in lines:
            line = ANSI_ESCAPE.sub('', raw_line).strip()
            parts = line.split()
            if len(parts) < 3:
                self.controllers_list.addItem(QtWidgets.QListWidgetItem(line))
                continue

            name = parts[0]
            status = parts[-1]
            display = f"{name}    {status}"
            item = QtWidgets.QListWidgetItem(display)

            status_lower = status.lower()
            if 'active' in status_lower:
                item.setBackground(QtGui.QColor(144, 238, 144))
            elif 'inactive' in status_lower:
                item.setBackground(QtGui.QColor(255, 200, 124))
            elif 'unconfigured' in status_lower:
                item.setBackground(QtGui.QColor(200, 200, 200))

            self.controllers_list.addItem(item)

    # ------------------------------------------------------------------
    # Button state management
    # ------------------------------------------------------------------

    def _ensure_system_running(self):
        """Return True if the 'system' tool is configured and fully ready."""
        system_tool = self.tools.get('system')
        if not system_tool:
            QtWidgets.QMessageBox.warning(
                self,
                'Full System Required',
                'Full system tool is not configured in the launcher.',
            )
            return False

        if system_tool.get('command') == FULL_SYSTEM_BASE_CMD:
            QtWidgets.QMessageBox.warning(
                self,
                'System Command Incomplete',
                'Full system command is not configured. Select a simulation world or complete the system command before starting other tools.',
            )
            return False

        if not self._is_running(system_tool):
            QtWidgets.QMessageBox.warning(
                self,
                'Full System Required',
                'Start the full system first before launching other tools.',
            )
            return False

        if not self._is_ros_system_ready():
            QtWidgets.QMessageBox.warning(
                self,
                'Full System Loading',
                'Full system is starting but not fully initialized yet. Please wait a moment and try again.',
            )
            return False

        return True

    def _update_tool_buttons(self, name):
        """Set start/stop/restart button enabled state for a single tool."""
        tool = self.tools[name]
        running = self._is_running(tool)

        # Stop enabled only when running
        tool['stop_button'].setEnabled(running)

        # Start enabled when not running (and system ready for non-system tools)
        start_enabled = not running
        system_tool = self.tools.get('system')
        system_is_ready = bool(system_tool and self._is_running(system_tool) and self._is_ros_system_ready())

        if name != 'system':
            start_enabled = start_enabled and system_is_ready

        tool['start_button'].setEnabled(start_enabled)

        # Restart enabled when system ready (for non-system), and not during system-not-ready
        restart_btn = tool.get('restart_button')
        if restart_btn is not None:
            if name == 'system':
                restart_btn.setEnabled(True)
            else:
                restart_btn.setEnabled(system_is_ready)

        # Stop All button enabled only when full system is fully ready (same behavior as before)
        stop_all_btn = self.findChild(QtWidgets.QPushButton, 'button_reset_all')
        if stop_all_btn is not None:
            stop_all_btn.setEnabled(system_is_ready)

    def _update_all_buttons(self):
        """Refresh start/stop/restart/stop-all buttons for every registered tool."""
        for name in self.tools.keys():
            self._update_tool_buttons(name)

    @staticmethod
    def _is_running(tool):
        return bool(tool['process'] and tool['process'].poll() is None)

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
