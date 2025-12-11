#!/usr/bin/env python3
"""PyQt5 GUI with per-tool start/stop buttons for the full system, Gazebo, RViz, and related tools."""

import os
import shlex
import signal
import subprocess
import sys
import re
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
    """GUI that starts/stops each ROS 2 tool in the background."""

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

        # Reset state
        self._reset_timer = None
        self._reset_countdown = 0
        self._reset_running_tools = []
        self._reset_still_running = []

        self._load_ui()
        self._set_branding_image()

        # Optional command-line display (may be None if not in UI)
        self.command_line = self.findChild(QtWidgets.QLineEdit, 'line_command')

        self._setup_world_selector()
        self._update_launch_command()

        # Register tools (buttons + status labels must exist in UI)
        self._register_tool(
            name='system',
            command=self._build_launch_command(),
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
            command=PERCEPTION_CMD,
            start_button='button_perception_start',
            stop_button='button_perception_stop',
            status_label='label_perception_status',
        )

        # Periodic monitor for processes launched by this GUI
        self.monitor_timer = QtCore.QTimer(self)
        self.monitor_timer.timeout.connect(self._cleanup_finished_processes)
        self.monitor_timer.start(1000)

        # Wire Reset button
        reset_btn = self.findChild(QtWidgets.QPushButton, 'button_reset_all')
        if reset_btn is not None:
            reset_btn.clicked.connect(self._reset_all_processes)

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
        command = FULL_SYSTEM_BASE_CMD
        if self._current_world_path:
            command = f"{command} simulation_world:={shlex.quote(self._current_world_path)}"
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
            process.wait(timeout=5)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
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
    # Reset-all logic (GUI-launched + external ROS/Gazebo/MoveIt processes)
    # ------------------------------------------------------------------

    def _reset_all_processes(self):
        """Stop all GUI-managed tools and then kill external ROS/Gazebo/MoveIt processes."""
        self._cleanup_finished_processes()

        # Phase 1: send SIGINT to all running tools started via this GUI
        running_tools = []
        for name, tool in self.tools.items():
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
            # No GUI-owned processes, but still clean up any stray ROS-related processes
            self._kill_external_ros_processes()
            QtWidgets.QMessageBox.information(
                self,
                'Reset Complete',
                'No GUI processes running. ROS-related processes have been cleaned up.',
            )
            return

        # Start countdown timer to wait for graceful exit
        self._reset_countdown = 5.0  # seconds
        if self._reset_timer is None:
            self._reset_timer = QtCore.QTimer(self)
            self._reset_timer.timeout.connect(self._reset_countdown_tick)
        if not self._reset_timer.isActive():
            self._reset_timer.start(500)  # 500 ms

        # Initial tick to update state immediately
        self._reset_countdown_tick()

    def _reset_countdown_tick(self):
        """Called periodically to check process status and complete reset."""
        still_running = []
        for name, tool in self._reset_still_running:
            if tool['process'] and tool['process'].poll() is None:
                still_running.append((name, tool))
        self._reset_still_running = still_running

        # All GUI-owned processes have exited
        if not self._reset_still_running:
            if self._reset_timer and self._reset_timer.isActive():
                self._reset_timer.stop()

            # Kill any remaining external ROS-related processes
            self._kill_external_ros_processes()
            self._cleanup_finished_processes()

            
            return

        # Countdown reached 0: force-kill remaining GUI-owned processes
        if self._reset_countdown <= 0:
            if self._reset_timer and self._reset_timer.isActive():
                self._reset_timer.stop()

            force_killed = 0
            for name, tool in self._reset_still_running:
                if tool['process'] and tool['process'].poll() is None:
                    try:
                        os.killpg(os.getpgid(tool['process'].pid), signal.SIGKILL)
                        force_killed += 1
                        self._set_tool_status(tool, 'Force-killed.')
                    except Exception:
                        pass
                    finally:
                        tool['process'] = None

            # After force-killing GUI-owned processes, also clean up external ROS processes
            self._kill_external_ros_processes()
            self._cleanup_finished_processes()
            
            return

        # Decrement countdown and keep waiting
        self._reset_countdown -= 0.5

    def _kill_external_ros_processes(self):
        """Force-kill common ROS2 / Gazebo / MoveIt / perception processes not spawned by the GUI."""
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
        # Skip if a check is already running
        if self._ros_check_process is not None and self._ros_check_process.state() == QtCore.QProcess.Running:
            return

        # Build the ros2 command with setup script sourcing
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
                stdout = self._ros_check_process.readAllStandardOutput().data().decode('utf-8', errors='ignore')
                ready = (exit_code == 0 and len(stdout.strip()) > 0)
        except Exception:
            ready = False
        finally:
            if self._ros_check_process is not None:
                self._ros_check_process.deleteLater()
            self._ros_check_process = None

        # Update cached flag and refresh UI if value changed
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

        # Avoid overlapping runs
        if self._controllers_process is not None and self._controllers_process.state() == QtCore.QProcess.Running:
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
            # Remove color codes
            line = ANSI_ESCAPE.sub('', raw_line).strip()

            # Expected format: <name> <type> <status>
            parts = line.split()
            if len(parts) < 3:
                # Fallback: list raw cleaned line
                self.controllers_list.addItem(QtWidgets.QListWidgetItem(line))
                continue

            name = parts[0]
            status = parts[-1]

            # Display only: "<name>    <status>"
            display = f"{name}    {status}"

            item = QtWidgets.QListWidgetItem(display)

            # Color by status
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
        """Return True if the 'system' tool is configured and fully ready.

        Checks that the process is running AND ROS appears responsive. Shows a warning
        and blocks starting other tools otherwise.
        """
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
        """Set start/stop button enabled state for a single tool."""
        tool = self.tools[name]
        running = self._is_running(tool)

        # Stop button enabled only when running
        tool['stop_button'].setEnabled(running)

        # Default start enabled if not running
        start_enabled = not running

        # For non-system tools, require the system to be fully ready
        if name != 'system':
            system_tool = self.tools.get('system')
            system_is_ready = bool(
                system_tool and self._is_running(system_tool) and self._is_ros_system_ready()
            )
            start_enabled = start_enabled and system_is_ready

        tool['start_button'].setEnabled(start_enabled)

        # Update Reset button: enabled only when full system is fully ready
        reset_btn = self.findChild(QtWidgets.QPushButton, 'button_reset_all')
        if reset_btn is not None:
            system_tool = self.tools.get('system')
            reset_enabled = bool(
                system_tool and self._is_running(system_tool) and self._is_ros_system_ready()
            )
            reset_btn.setEnabled(reset_enabled)

    def _update_all_buttons(self):
        """Refresh start/stop/reset buttons for every registered tool."""
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
