# Power Monitoring Troubleshooting Guide

## Common Issues and Solutions

### Issue 1: "No joint states received" or NaN values

**Symptoms:**
```
[INFO] [power_monitor_node]: Power: nanW | Current: nanA | Efficiency: 0.0% | Energy: nanWh
[WARN] [power_monitor_node]: No joint states received. Is the simulation running?
```

**Cause:** The power monitor node is not receiving joint state data.

**Solutions:**

1. **Start simulation FIRST, then power monitor:**
   ```bash
   # Terminal 1: Start simulation
   ros2 launch arm_system_bringup full_system.launch.py
   # Wait 5-10 seconds for full startup

   # Terminal 2: Start power monitor
   ros2 launch arm_control power_monitoring.launch.py
   ```

2. **Check if joint_states topic exists:**
   ```bash
   ros2 topic list | grep joint_states
   # Should show: /joint_states

   ros2 topic hz /joint_states
   # Should show: ~50 Hz
   ```

3. **Check topic contents:**
   ```bash
   ros2 topic echo /joint_states --once
   # Should show joint names, positions, velocities, efforts
   ```

4. **If joint_states not publishing:**
   - Simulation may have crashed - check Gazebo logs
   - Controllers may not be loaded - run: `ros2 control list_controllers`
   - Wait longer for simulation startup (can take 10-15 seconds)

---

### Issue 2: GUI not showing data

**Symptoms:** Power monitor GUI opens but shows 0W and flat graphs.

**Solutions:**

1. **Verify simulation is running:**
   ```bash
   ros2 topic hz /joint_states
   ```

2. **Check ROS 2 node communication:**
   ```bash
   ros2 node list
   # Should include: /power_monitor_gui_node
   ```

3. **Restart GUI:**
   - Close GUI window
   - Restart: `ros2 run arm_gui_tools power_monitor_gui`

4. **Check for Python errors:**
   - Look in terminal where GUI was launched
   - Common issues: matplotlib not installed, PyQt5 issues

---

### Issue 3: Very high or unrealistic power values

**Symptoms:** Power shows >5000W, current >100A, or other unrealistic values.

**Possible causes:**

1. **Simulation instability:**
   - Check Gazebo for robot oscillations or collisions
   - Reset simulation: Ctrl+C and restart

2. **Controller gains too high:**
   - Reduce PID gains in `config/controllers.yaml`
   - Check joint limits in URDF

3. **Torque spikes from collisions:**
   - Check for unexpected collisions in Gazebo
   - Review collision geometry in robot URDF

**Debugging:**
```bash
# Monitor joint efforts (torques)
ros2 topic echo /joint_states | grep effort

# If torques are extremely high (>100 Nm), simulation has issues
```

---

### Issue 4: CSV logging not working

**Symptoms:** No CSV file created, or file is empty.

**Solutions:**

1. **Check file permissions:**
   ```bash
   # Try writing to home directory instead of /tmp
   ros2 launch arm_control power_monitoring.launch.py \
     log_to_file:=true \
     log_file_path:=$HOME/power_log.csv
   ```

2. **Check parameter is set:**
   ```bash
   ros2 param list /power_monitor_node
   # Should show: log_to_file

   ros2 param get /power_monitor_node log_to_file
   # Should show: true
   ```

3. **Check disk space:**
   ```bash
   df -h $HOME
   ```

4. **Verify node is receiving data:**
   - If no joint states, CSV will be empty (headers only)

---

### Issue 5: Power values seem too low

**Symptoms:** System shows 5-10W even during fast motion.

**Possible causes:**

1. **Effort (torque) not being published:**
   ```bash
   ros2 topic echo /joint_states
   # Check if 'effort' array has non-zero values
   ```

2. **Actuator specs not loaded:**
   - Check logs for error loading `actuator_specs.yaml`
   - Verify file exists: `ls install/arm_control/share/arm_control/config/actuator_specs.yaml`

3. **Joint names mismatch:**
   - Power calculator expects specific joint names
   - Check joint names in `/joint_states` match those in `actuator_specs.yaml`

**Debug with test script:**
```bash
ros2 run arm_control test_power_monitor.py
# If this shows reasonable values, problem is with real-time data
```

---

### Issue 6: "Failed to load actuator specs" error

**Symptoms:**
```
[ERROR] [power_monitor_node]: Cannot find actuator_specs.yaml!
```

**Solutions:**

1. **Rebuild package:**
   ```bash
   colcon build --packages-select arm_control --allow-overriding arm_control
   source install/setup.bash
   ```

2. **Verify file exists:**
   ```bash
   find install/ -name actuator_specs.yaml
   # Should find: install/arm_control/share/arm_control/config/actuator_specs.yaml
   ```

3. **Check package share directory:**
   ```bash
   ros2 pkg prefix arm_control
   # Should show: /path/to/workspace/install/arm_control
   ```

---

### Issue 7: Import errors (Python modules not found)

**Symptoms:**
```
ModuleNotFoundError: No module named 'PyQt5'
ModuleNotFoundError: No module named 'matplotlib'
```

**Solutions:**

**For PyQt5 (GUI):**
```bash
sudo apt install python3-pyqt5
pip3 install PyQt5
```

**For matplotlib (graphs):**
```bash
sudo apt install python3-matplotlib
pip3 install matplotlib
```

**For YAML:**
```bash
pip3 install pyyaml
```

**Verify installations:**
```bash
python3 -c "import PyQt5; print('PyQt5 OK')"
python3 -c "import matplotlib; print('matplotlib OK')"
python3 -c "import yaml; print('yaml OK')"
```

---

### Issue 8: GUI crashes or freezes

**Symptoms:** Power monitor GUI becomes unresponsive or crashes.

**Solutions:**

1. **Check ROS 2 is running:**
   ```bash
   ros2 node list
   # Should show active nodes
   ```

2. **Restart GUI with clean state:**
   ```bash
   # Kill any existing GUI processes
   pkill -f power_monitor_gui

   # Restart
   ros2 run arm_gui_tools power_monitor_gui
   ```

3. **Check for Qt errors:**
   - Look for Qt-related errors in terminal
   - Try setting: `export QT_DEBUG_PLUGINS=1`

4. **Memory issues:**
   - Close other applications
   - Reduce update rate: `ros2 launch arm_control power_monitoring.launch.py update_rate:=5.0`

---

## Verification Checklist

Use this checklist to verify system is working:

- [ ] Workspace built successfully: `colcon build --packages-select arm_control arm_gui_tools`
- [ ] Package sourced: `source install/setup.bash`
- [ ] Test script runs: `ros2 run arm_control test_power_monitor.py` (shows power calculations)
- [ ] Simulation starts: `ros2 launch arm_system_bringup full_system.launch.py`
- [ ] Joint states publishing: `ros2 topic hz /joint_states` (shows ~50 Hz)
- [ ] Controllers active: `ros2 control list_controllers` (arm_controller active)
- [ ] Power monitor node starts: `ros2 run arm_control power_monitor_node.py` (no errors)
- [ ] Topics published: `ros2 topic list | grep power_monitor` (shows 4 topics)
- [ ] GUI starts: `ros2 run arm_gui_tools power_monitor_gui` (window opens)
- [ ] Motion generates data: `ros2 run arm_control example.py` (power values change)

---

## Debug Commands

**Check system status:**
```bash
# ROS 2 nodes
ros2 node list

# Topics
ros2 topic list

# Joint states rate
ros2 topic hz /joint_states

# Controllers
ros2 control list_controllers

# Power monitor parameters
ros2 param list /power_monitor_node
```

**Monitor real-time data:**
```bash
# Total power
ros2 topic echo /power_monitor/total_power

# Statistics
ros2 topic echo /power_monitor/statistics

# Joint states (raw)
ros2 topic echo /joint_states
```

**Check for errors:**
```bash
# Node logs
ros2 run arm_control power_monitor_node.py 2>&1 | grep ERROR

# Package installation
ros2 pkg list | grep arm_control
ros2 pkg list | grep arm_gui_tools
```

---

## Getting Help

If you're still having issues:

1. **Check the full documentation:**
   - [POWER_MONITORING.md](POWER_MONITORING.md) - Complete system documentation
   - [POWER_MONITORING_QUICKSTART.md](POWER_MONITORING_QUICKSTART.md) - Quick start guide

2. **Run the test script first:**
   ```bash
   ros2 run arm_control test_power_monitor.py
   ```
   If this works, the calculations are correct and the issue is with real-time data.

3. **Verify simulation works independently:**
   ```bash
   ros2 launch arm_system_bringup full_system.launch.py
   ros2 run arm_control example.py
   ```
   Power monitoring requires a working simulation.

4. **Check GitHub issues:**
   - Repository: https://github.com/widemic/ldr-humanoid-arm-system
   - Look for similar issues or create a new one

---

**Last Updated:** 2025-11-21
**Version:** 1.0
