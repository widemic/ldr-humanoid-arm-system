# Troubleshooting: Object Tracking Nu Funcționează

## Diagnostic Rapid

### 1. Rulează Script-ul de Test

```bash
cd /home/andrei/ros2_ws/ldr-humanoid-arm-system
./test_perception.sh
```

**Verifică output-ul:** Toate check-urile ar trebui să fie ✓ (verde).

### 2. Verificare Manuală - Pas cu Pas

#### Pas 1: Verifică că Gazebo și Camera Rulează

```bash
# Verifică topic-uri camera
ros2 topic list | grep camera

# Ar trebui să vezi:
#  /camera/camera_info
#  /camera/depth/image_raw
#  /camera/depth/points    ← IMPORTANT!
#  /camera/image_raw
```

**Dacă NU vezi `/camera/depth/points`:**
- Camera depth nu e pornită în Gazebo
- Verifică că ai lansat: `ros2 launch arm_system_bringup moveit_gazebo.launch.py`

#### Pas 2: Verifică Point Cloud Are Date

```bash
# Verifică dimensiuni
ros2 topic echo /camera/depth/points --field width --once
ros2 topic echo /camera/depth/points --field height --once

# Verifică rate
ros2 topic hz /camera/depth/points

# Ar trebui să vezi: ~30 Hz, width=640, height=480 (sau similar)
```

**Dacă width/height = 0:**
- Camera nu generează imagini
- Verifică în Gazebo GUI că ai obiectele vizibile în fața camerei

#### Pas 3: Verifică Tracker-ul Rulează

```bash
# Lista noduri
ros2 node list

# Ar trebui să vezi:
#  /dynamic_object_tracker

# Info despre nod
ros2 node info /dynamic_object_tracker
```

**Dacă NU vezi `/dynamic_object_tracker`:**

```bash
# Lansează manual pentru debugging
source install/setup.bash
ros2 run arm_perception dynamic_object_tracker.py --ros-args --log-level info
```

#### Pas 4: Verifică Log-urile Tracker-ului

```bash
# Vezi log-uri în timp real
ros2 topic echo /rosout | grep dynamic_object_tracker
```

**Ce să cauți:**
```
[INFO] [dynamic_object_tracker]: Dynamic Object Tracker started
[INFO] [dynamic_object_tracker]: Frame 30: XXXX valid points
[INFO] [dynamic_object_tracker]: Initialized N objects
[INFO] [dynamic_object_tracker]: Object X moved 0.XXXm to [x, y, z]
```

**Dacă NU vezi "Frame X: XXXX valid points":**
- Point cloud NU ajunge la tracker
- Problema e la topic subscription

**Dacă vezi "Frame X: 0 valid points":**
- Point cloud e gol sau toate punctele sunt filtrate
- Verifică parametri filtering

#### Pas 5: Verifică Obiecte Detectate

```bash
# Vezi markerii de debug
ros2 topic echo /tracked_objects

# Vezi collision objects
ros2 topic echo /planning_scene --field world.collision_objects
```

**Dacă `/tracked_objects` e gol:**
- Nu sunt obiecte detectate ÎN fața camerei
- Plasează un obiect (Box) în Gazebo în fața camerei

#### Pas 6: Vizualizare în RViz

**Setup RViz:**
1. Add → **MarkerArray**
   - Topic: `/tracked_objects`
   - Ar trebui să vezi: Bounding boxes verzi

2. Add → **PointCloud2**
   - Topic: `/camera/depth/points`
   - Size: 0.01
   - Ar trebui să vezi: Point cloud-ul camerei

3. Add → **PlanningScene**
   - Enable: Scene Geometry
   - Ar trebui să vezi: Collision objects

**Testează mișcare:**
- În Gazebo: Insert → Box → plasează în fața camerei
- Mută box-ul cu mouse-ul
- În RViz: Marker-ul verde ar trebui să URMĂREASCĂ box-ul

## Probleme Comune

### Problema 1: "Frame X: 0 valid points"

**Cauză:** Filtrarea elimină toate punctele.

**Soluție:**
```bash
# Editează perception.launch.py - reduce filtering
min_object_height: 0.0    # de la 0.02
cluster_tolerance: 0.1    # de la 0.05
min_cluster_size: 10      # de la 50
```

```bash
# Rebuild și restart
colcon build --packages-select arm_perception
source install/setup.bash
ros2 launch arm_perception perception.launch.py
```

### Problema 2: Tracker-ul NU primește point cloud

**Diagnostic:**
```bash
# Verifică topic subscription
ros2 node info /dynamic_object_tracker | grep Subscriptions

# Ar trebui să vezi:
#  /camera/depth/points: sensor_msgs/msg/PointCloud2
```

**Dacă NU vezi subscription-ul:**
- Node-ul nu s-a lansat corect
- Rebuild package-ul

**Soluție:**
```bash
# Kill node-ul
ros2 node kill /dynamic_object_tracker

# Rulează manual cu debugging
ros2 run arm_perception dynamic_object_tracker.py \
    --ros-args \
    --log-level debug \
    --param use_sim_time:=true
```

### Problema 3: "scipy not available" Error

**Cauză:** scipy nu e instalat.

**Soluție:**
```bash
pip3 install scipy
```

**NU e critic:** Tracker-ul poate rula FĂRĂ scipy (folosește calcul manual de distanță).

### Problema 4: Obiecte Detectate, Dar NU Apar în Planning Scene

**Diagnostic:**
```bash
# Verifică că planning_scene publică
ros2 topic hz /planning_scene

# Verifică collision objects
ros2 topic echo /planning_scene --field world.collision_objects[0].id
```

**Dacă vezi "tracked_object_X" în collision_objects:**
- ✅ Tracker-ul funcționează!
- Problema e la vizualizare în RViz

**Dacă NU vezi collision objects:**
```bash
# Verifică log-uri pentru erori
ros2 topic echo /rosout | grep ERROR
```

### Problema 5: "Fantome" Încă Persistă

**Cauză:** Ai DOUĂ sisteme de collision detection care se bat:
- dynamic_object_tracker (REMOVE + ADD)
- planning_scene_updater (OctoMap - nu șterge instant)

**Soluție:** Folosește DOAR unul!

**Opțiunea A: Doar Object Tracking (Recomandat pentru obiecte mobile)**
```bash
ros2 launch arm_perception perception.launch.py \
    enable_octomap:=false \
    enable_object_tracking:=true
```

**Opțiunea B: Doar OctoMap (Pentru scene statice)**
```bash
ros2 launch arm_perception perception.launch.py \
    enable_octomap:=true \
    enable_object_tracking:=false
```

**Opțiunea C: Ambele (EXPERIMENTAL)**
- OctoMap pentru background static (pereți, masă)
- Object Tracking pentru obiecte mobile
- Risc de conflict între cele două!

## Debugging Avansat

### Vezi Exact Ce Publică Tracker-ul

```bash
# Monitor planning_scene updates
ros2 topic echo /planning_scene | grep -A 20 "collision_objects"
```

**Ar trebui să vezi:**
```yaml
collision_objects:
  - operation: 1    # ← REMOVE
    id: "tracked_object_0"
  - operation: 0    # ← ADD
    id: "tracked_object_0"
    primitives:
      - type: 1     # BOX
        dimensions: [0.1, 0.1, 0.1]
    primitive_poses:
      - position:
          x: 1.0
          y: 0.5
          z: 0.3
```

### Profile Performanță

```bash
# Verifică CPU usage
top -p $(pgrep -f dynamic_object_tracker)

# Ar trebui: 10-30% CPU (single core)
```

**Dacă CPU > 50%:**
- Point cloud prea mare (subsample-ază)
- Clustering foarte lent (crește cluster_tolerance)

## Checklist Final

✅ **Gazebo rulează cu camera depth**
```bash
ros2 topic hz /camera/depth/points   # ~30 Hz
```

✅ **Point cloud are date**
```bash
ros2 topic echo /camera/depth/points --field width --once  # > 0
```

✅ **Tracker rulează**
```bash
ros2 node list | grep dynamic_object_tracker  # există
```

✅ **Tracker primește date**
```bash
ros2 topic echo /rosout | grep "Frame"  # vezi "Frame X: YYYY points"
```

✅ **Obiecte detectate**
```bash
ros2 topic echo /rosout | grep "Initialized"  # vezi "Initialized N objects"
```

✅ **Collision objects publicate**
```bash
ros2 topic echo /planning_scene --field world.collision_objects[0].id  # "tracked_object_X"
```

✅ **Vizualizare în RViz**
- MarkerArray `/tracked_objects`: Vede bounding boxes verzi
- PointCloud2 `/camera/depth/points`: Vede point cloud
- PlanningScene: Vede collision objects

## Comenzi Utile

```bash
# Restart doar tracker-ul
ros2 node kill /dynamic_object_tracker
ros2 run arm_perception dynamic_object_tracker.py

# Verifică toate topic-uri perception
ros2 topic list | grep -E "(camera|tracked|planning)"

# Monitor log-uri în timp real
ros2 topic echo /rosout --field msg | grep -i tracker

# Dump planning scene complet
ros2 topic echo /planning_scene > planning_scene_dump.txt

# Verifică TF tree (pentru debugging frame-uri)
ros2 run tf2_tools view_frames
```

## Contact și Raportare Bug-uri

Dacă problema persistă:

1. **Colectează informații:**
```bash
# Salvează output
ros2 topic list > topics.txt
ros2 node list > nodes.txt
ros2 node info /dynamic_object_tracker > tracker_info.txt
ros2 topic echo /rosout > logs.txt   # Ctrl+C după 10 secunde
```

2. **Verifică fișierele:**
- [dynamic_object_tracker.py](src/perception/arm_perception/scripts/dynamic_object_tracker.py)
- [perception.launch.py](src/perception/arm_perception/launch/perception.launch.py)

3. **Raportează la:** https://github.com/widemic/ldr-humanoid-arm-system/issues

---

**Autor:** Debug guide pentru LDR Humanoid Arm System
**Data:** 2025-11-19
