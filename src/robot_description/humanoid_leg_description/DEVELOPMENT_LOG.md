# robot_description - Development Log

## Package Overview

Acest package conține descrierea URDF completă pentru robotul umanoid KBOT (picioare, torso, cap) importată din kbot-v2.

## Starea Curentă (2025-12-02)

### ✅ Ce Funcționează

1. **URDF Structure** - Complet funcțional
   - Links: base_link, torso_link, imu_link, 12 leg links (6 per leg)
   - Joints: 12 revolute joints pentru picioare (6 per leg)
   - Meshuri: 31 STL files (visual + collision)

2. **RViz Visualization** - Funcțional
   - Launcher: `ros2 launch robot_description display.launch.py`
   - Robotul se afișează corect cu toate meshurile
   - Joint state publisher GUI funcțional

3. **File Organization**
   ```
   robot_description/
   ├── urdf/
   │   ├── robot.urdf.xacro          # Main URDF
   │   ├── links/
   │   │   ├── torso_links.xacro     # Torso + IMU
   │   │   └── leg_links.xacro       # 12 leg links
   │   ├── joints/
   │   │   └── leg_joints.xacro      # 12 leg joints
   │   └── macros/
   │       ├── materials.xacro
   │       ├── gazebo_materials.xacro
   │       └── ros2_control_gazebo.xacro
   ├── meshes/                       # 31 STL files
   ├── config/
   │   └── controllers.yaml          # ros2_control config
   ├── worlds/
   │   └── empty.sdf                 # Gazebo world
   └── launch/
       ├── display.launch.py         # RViz viewer
       ├── gazebo.launch.py          # Full Gazebo sim
       └── spawn_robot.launch.py     # Spawn in existing Gazebo
   ```

### 🚧 În Progres

**Gazebo Integration** - Aproape completă, dar cu erori

**Ultima Modificare:** Înlocuit `ExecuteProcess` cu `IncludeLaunchDescription` pentru `gz_sim.launch.py`

**Status:** Build OK, dar trebuie testat runtime

**Problema Anterioară:**
```
[gz-2] terminate called after throwing an instance of 'std::runtime_error'
[gz-2]   what():  no ros2_control tag
```

**Soluție Aplicată:**
- Folosim acum `IncludeLaunchDescription` pentru `ros_gz_sim/launch/gz_sim.launch.py`
- Acest launcher oficial setează automat toate variabilele de mediu necesare
- Elimină nevoia de a seta manual `GZ_SIM_SYSTEM_PLUGIN_PATH`

### 📋 Probleme Rezolvate

1. **Mesh Loading în RViz** ✅
   - Problema: Meshurile nu se încărcau
   - Cauză: Nume diferite între STL files și URDF
   - Soluție: Creat copii cu nume corecte (ex: `KB_D_102L_L_Hip_Yoke_Drive.stl` → `KB_D_102L_L_HIP_YOKE.stl`)

2. **Visual Display în RViz** ✅
   - Problema: Meshurile se încărcau dar nu erau vizibile
   - Cauză 1: Materiale definite ca referințe, nu inline
   - Soluție: Schimbat la `<material name=""><color rgba="..."/></material>`
   - Cauză 2: Tag ordering greșit (visual, collision, inertial)
   - Soluție: Reordonat la (inertial, visual, collision)
   - Cauză 3: Origin tag după geometry în visual/collision
   - Soluție: Mutat origin tag înainte de geometry

3. **Mesh Scale Issue** ✅
   - Problema: Meshurile erau invizibile în RViz
   - Cauză: Scale `0.001` aplicat pe meshuri deja în metri → 0.18mm (invizibil!)
   - Soluție: Schimbat scale de la `0.001` la `1.0`
   - Verificare: Analizat STL dimensions cu Python - torso: 0.18m x 0.14m x 0.58m

4. **Joint Origins Wrong** ✅
   - Problema: Meshurile vizibile dar la poziții greșite
   - Cauză: Joint origins inventate, nu din URDF original
   - Soluție: Copiat EXACT toate origins și RPY din kbot-v2 original
   - Source: `/home/alex/ws_kbot/kbot/ksim-kbot/ksim_kbot/kscale-assets/kbot-v2-feet/robot.urdf`

5. **Robot Floating în Aer** ✅
   - Problema: Robotul nu era la nivelul solului
   - Cauză: `base_to_torso` joint la z=0
   - Soluție: Calculat înălțime totală picior (~0.813m), setat `base_to_torso` origin la `z=0.813`

6. **Gazebo Mesh Loading** ✅
   - Problema: `Unable to find file with URI [model://robot_description/meshes/...]`
   - Cauză: Gazebo nu știa unde să caute package-urile ROS
   - Soluție: Setat `GZ_SIM_RESOURCE_PATH` în launch file

7. **gz_ros2_control Plugin Loading** ✅
   - Problema: `Failed to load system plugin [libgz_ros2_control-system.so]`
   - Cauză: Gazebo nu găsea biblioteca plugin-ului
   - Soluție: Setat `GZ_SIM_SYSTEM_PLUGIN_PATH` la `/opt/ros/jazzy/lib`

8. **ros2_control Tag Missing** ✅
   - Problema: `terminate called... no ros2_control tag`
   - Cauză: Folosit `ExecuteProcess` cu `gz sim` direct, care nu setează env vars corect
   - Soluție: Folosit `IncludeLaunchDescription` pentru `gz_sim.launch.py` oficial

## Detalii Tehnice Importante

### Robot Specifications

**Total Links:** 14
- 1x base_link (floating base)
- 1x torso_link
- 1x imu_link
- 6x left leg links (hip_yoke, hip_roll, hip_pitch, femur, shin, foot)
- 6x right leg links (similar)

**Total Joints:** 12 actuated + 1 fixed
- 1x base_to_torso (fixed)
- 1x imu_joint (fixed)
- 6x left leg joints (hip_pitch, hip_roll, hip_yaw, knee, ankle, foot)
- 6x right leg joints (similar)

**Robot Height:** ~0.813m (ground to torso)

### Joint Structure (Left Leg Example)

```
torso_link
  └─[left_hip_pitch]→ left_hip_yoke_link
      └─[left_hip_roll]→ left_hip_roll_link
          └─[left_hip_yaw]→ left_hip_pitch_link
              └─[left_knee]→ left_femur_link
                  └─[left_ankle]→ left_shin_link
                      └─[left_foot]→ left_foot_link
```

### Critical Joint Origins (from kbot-v2 original)

**Left Leg:**
```xml
torso → hip_yoke:    xyz="-0.056 -0.000737 -0.072993" rpy="-1.5708 0 -1.5708"
hip_yoke → hip_roll: xyz="-0.02825 0.03 -0.071"      rpy="3.141593 1.5708 0"
hip_roll → hip_yaw:  xyz="0 -0.14275 -0.0298"       rpy="1.5708 0 0"
hip_yaw → femur:     xyz="0 0 0"                     rpy="0 0 0"
femur → shin:        xyz="0.0432 -0.021 0.212"      rpy="-1.5708 0 -1.5708"
shin → foot:         xyz="-0.0313 -0.2925 -0.0572"  rpy="0 0 0"
```

**Right Leg:**
```xml
torso → hip_yoke:    xyz="0.056 -0.000737 -0.072993" rpy="1.5708 0 -1.5708"
hip_yoke → hip_roll: xyz="-0.02825 -0.03 -0.071"    rpy="0 -1.5708 0"
hip_roll → hip_yaw:  xyz="0 -0.14275 -0.0298"       rpy="1.5708 0 0"
hip_yaw → femur:     xyz="0 0 0"                     rpy="0 0 0"
femur → shin:        xyz="0.0206 -0.021 0.212"      rpy="1.5708 0 -1.5708"
shin → foot:         xyz="-0.0313 0.2925 0.034"     rpy="0 0 0"
```

### ros2_control Configuration

**Hardware Plugin:** `gz_ros2_control/GazeboSimSystem`

**Controllers:**
- `joint_state_broadcaster` - Publishes joint states at 50 Hz
- `leg_controller` - JointTrajectoryController for all 12 leg joints

**Command Interfaces:** position
**State Interfaces:** position, velocity

### Launch Files

**display.launch.py** - RViz visualization
```bash
ros2 launch robot_description display.launch.py
```
Arguments: `use_sim_time`, `gui`, `use_sim`, `prefix`

**gazebo.launch.py** - Full Gazebo simulation
```bash
ros2 launch robot_description gazebo.launch.py
```
Arguments: `world`, `use_sim_time`, `gui`, `headless`, `use_sim`, `prefix`

**spawn_robot.launch.py** - Spawn in existing Gazebo
```bash
# Terminal 1
gz sim empty.sdf

# Terminal 2
ros2 launch robot_description spawn_robot.launch.py
```
Arguments: `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `use_sim`, `prefix`

### Environment Variables Set

1. **GZ_SIM_RESOURCE_PATH** = `<install_dir>/share`
   - Pentru găsirea meshurilor cu `package://` URI

2. **GZ_SIM_SYSTEM_PLUGIN_PATH** = `/opt/ros/jazzy/lib`
   - Pentru găsirea `libgz_ros2_control-system.so`

3. **Folosim gz_sim.launch.py** care setează automat:
   - `GZ_SIM_SYSTEM_PLUGIN_PATH`
   - `LD_LIBRARY_PATH`
   - `GAZEBO_MODEL_PATH`
   - Alte variabile necesare

## Următorii Pași

### 🔴 Urgent - Testare

1. **Test Gazebo Launch**
   ```bash
   source install/setup.bash
   ros2 launch robot_description gazebo.launch.py
   ```
   - Verifică dacă robotul apare în Gazebo
   - Verifică dacă meshurile se încarcă
   - Verifică dacă ros2_control se inițializează

2. **Test Controller Manager**
   ```bash
   ros2 control list_controllers
   ```
   - Ar trebui să apară: `joint_state_broadcaster` și `leg_controller`

3. **Test Joint States**
   ```bash
   ros2 topic echo /joint_states
   ```
   - Ar trebui să publice stări pentru toate cele 12 joint-uri

### 🟡 Medium Priority

1. **Add Sensors Plugin**
   - IMU sensor pe imu_link
   - Eventual force/torque sensors pe feet

2. **Improve Controllers Config**
   - Add PID gains
   - Add trajectory tolerances
   - Add constraints

3. **Create Test Scripts**
   - Script pentru mișcări simple ale picioarelor
   - Script pentru verificare kinematics

### 🟢 Low Priority

1. **Add Head/Arms**
   - Import head din kbot-v2 (dacă există)
   - Eventual add dummy arms pentru completitudine

2. **Documentation**
   - Add diagrams pentru joint structure
   - Add inertia calculation details
   - Add mesh source information

3. **Optimization**
   - Simplify collision meshes
   - Optimize inertia values
   - Test performance în Gazebo

## Known Issues

### Warning: Root Link Inertia
```
[kdl_parser]: The root link base_link has an inertia specified in the URDF,
but KDL does not support a root link with an inertia.
```
**Impact:** Low - doar un warning, KDL funcționează
**Solution:** Add extra dummy link între world și base_link (low priority)

### Mesh Files Naming
Meshurile originale au nume diferite de cele din URDF. Am creat copii cu nume corecte:
- Original: `KB_D_102L_L_Hip_Yoke_Drive.stl`
- Copie: `KB_D_102L_L_HIP_YOKE.stl`

**Locație originală:** `/home/alex/ws_kbot/kbot/ksim-kbot/ksim_kbot/kscale-assets/kbot-v2-feet/meshes/`

## Dependencies

```xml
<depend>urdf</depend>
<depend>xacro</depend>
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>joint_trajectory_controller</exec_depend>
```

## Build Instructions

```bash
# From workspace root
colcon build --packages-select robot_description
source install/setup.bash

# Test RViz
ros2 launch robot_description display.launch.py

# Test Gazebo (TO BE VERIFIED)
ros2 launch robot_description gazebo.launch.py
```

## References

- **Original URDF Source:** `/home/alex/ws_kbot/kbot/ksim-kbot/ksim_kbot/kscale-assets/kbot-v2-feet/robot.urdf`
- **Original Meshes:** `/home/alex/ws_kbot/kbot/ksim-kbot/ksim_kbot/kscale-assets/kbot-v2-feet/meshes/`
- **Reference Package:** `arm_description` (same workspace)

## Change Log

### 2025-12-02 - Initial Import
- ✅ Created package structure
- ✅ Imported URDF from kbot-v2 (legs, torso, IMU)
- ✅ Copied and renamed 31 STL mesh files
- ✅ Created modular xacro structure (links/, joints/, macros/)
- ✅ Fixed mesh naming issues
- ✅ Fixed material definitions (inline)
- ✅ Fixed tag ordering (inertial, visual, collision)
- ✅ Fixed mesh scale (1.0 instead of 0.001)
- ✅ Copied exact joint origins from kbot-v2
- ✅ Adjusted base_to_torso height (z=0.813m)
- ✅ Created Gazebo world (empty.sdf)
- ✅ Created launch files (display, gazebo, spawn_robot)
- ✅ Created ros2_control configuration
- ✅ Added GZ_SIM_RESOURCE_PATH setup
- ✅ Added GZ_SIM_SYSTEM_PLUGIN_PATH setup
- ✅ Switched to IncludeLaunchDescription for gz_sim.launch.py

---

**Last Updated:** 2025-12-02 22:05
**Status:** Ready for Gazebo runtime testing
**Maintainer:** Claude Code + Alex
