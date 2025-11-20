# Ghid: Ștergere Automată Obiecte Mutate în OctoMap

## Problemă Rezolvată

**Înainte:** Când muți un obiect în Gazebo, OctoMap păstra "fantoma" poziției vechi + noul obiect.

**Acum:** OctoMap detectează spațiul liber și șterge automat poziția veche când obiectul se mută.

## Cum Funcționează

### 1. Raycasting și Free Space Detection

```
Camera → → → → → Obstacle
  |  All voxels between camera and obstacle = FREE
  ↓
[FREE][FREE][FREE][OCCUPIED]
```

Când obiectul se mută:
```
Camera → → → → → [empty] → → New Position
  |  Old position acum e marcată FREE
  ↓
[FREE][FREE][FREE][FREE][FREE][OCCUPIED]
                              ↑ new position
```

### 2. Actualizări Probabilistice

OctoMap folosește **sensor model probabilistic**:

```yaml
sensor_model/hit: 0.7    # +70% probabilitate când detectezi obstacol
sensor_model/miss: 0.4   # -60% probabilitate când vezi liber
sensor_model/min: 0.12   # Limită minimă (sub acest prag = FREE)
sensor_model/max: 0.97   # Limită maximă (peste acest prag = OCCUPIED)
```

**Exemplu:** Un voxel la poziția veche a obiectului:
- **Frame 1-10:** Obstacol detectat → probabilitate crește la 0.97 (OCCUPIED)
- **Frame 11:** Obiect mutat, camera vede prin → probabilitate scade la 0.37
- **Frame 12:** Camera vede din nou liber → probabilitate scade la -0.23 → clamped la 0.12
- **Frame 13:** Sub threshold 0.12 → voxelul e marcat **FREE** și șters din vizualizare

## Configurări Cheie

### [sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml)

```yaml
# CRITICAL pentru clearing
publish_free_space: true     # Publică free space markers
max_update_rate: 5.0         # Update la fiecare 200ms
```

### [octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml)

```yaml
# MUST HAVE pentru dynamic scenes
publish_free_space: true     # Marchează voxeli liberi
latch: false                 # Nu latch - permite updates continue

# Sensor model - controlează cât de repede se șterg voxelii
sensor_model/hit: 0.7        # Cât de repede devine OCCUPIED
sensor_model/miss: 0.4       # Cât de repede devine FREE
sensor_model/min: 0.12       # Threshold pentru FREE
sensor_model/max: 0.97       # Threshold pentru OCCUPIED
```

### [planning_scene_updater.py](src/perception/arm_perception/scripts/planning_scene_updater.py)

```python
planning_scene.is_diff = True  # Differential updates
# FREE voxels în OctoMap vor șterge OCCUPIED voxels vechi din planning scene
```

## Testare

### Test 1: Mutare Obiect în Gazebo

```bash
# Terminal 1: Lansează sistemul
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Terminal 2: Lansează percepția (după 20s)
ros2 launch arm_perception perception.launch.py

# Terminal 3: Monitorizează OctoMap
ros2 topic hz /octomap_binary
```

**În Gazebo:**
1. Plasează un obiect (cub, sferă) în fața camerei
2. Așteaptă 2-3 secunde să apară în OctoMap (RViz)
3. Mută obiectul într-o altă poziție
4. **Observă:** Vechea poziție dispare în 1-2 secunde

### Test 2: Verificare Free Space

```bash
# Verifică că free space este publicat
ros2 topic echo /octomap_full --field data

# Vezi mesajele de la planning scene updater
ros2 topic echo /planning_scene --field is_diff
# Ar trebui să fie: true
```

### Test 3: Vizualizare în RViz

**Setup RViz:**
1. Add Display → **MarkerArray**
   - Topic: `/occupied_cells_vis_array` (voxeli ocupați - roșii/colorați)
2. Add Display → **MarkerArray**
   - Topic: `/free_cells_vis_array` (voxeli liberi - verzi, opțional)
3. Add Display → **PlanningScene**
   - Topic: `/planning_scene`
   - Enable: Scene Geometry

**Test:**
- Mută un obiect în Gazebo
- Vei vedea voxelii vechi dispărând treptat (devine verde apoi dispare)
- Noua poziție apare aproape instant

## Ajustări de Performanță

### Clearing Mai Rapid (Obiecte Dispar Mai Repede)

```yaml
# În octomap_server.yaml
sensor_model/miss: 0.5       # Crește de la 0.4 (decay mai rapid)
sensor_model/min: 0.15       # Crește de la 0.12 (threshold mai mare)

# În sensors_3d.yaml
max_update_rate: 10.0        # Crește de la 5.0 (updates mai dese)
```

**Trade-off:** Mai mult CPU, posibile false negatives (obiecte reale șterse prea repede)

### Clearing Mai Lent (Mai Stabil, Mai Puțin Noise)

```yaml
# În octomap_server.yaml
sensor_model/miss: 0.3       # Reduce de la 0.4 (decay mai lent)
sensor_model/min: 0.10       # Reduce de la 0.12 (threshold mai mic)

# În sensors_3d.yaml
max_update_rate: 2.0         # Reduce de la 5.0 (updates mai rare)
```

**Trade-off:** Mai puțin CPU, dar "fantome" persistă mai mult timp

### Recommended (Echilibru)

Setările curente sunt optimizate pentru echilibru:
- **5 Hz update rate** - Destul de rapid pentru dynamic scenes
- **0.4 miss probability** - Clearing moderat (2-3 frame-uri)
- **0.12 min threshold** - Bună separare între FREE/OCCUPIED

## Parametri Sensor Model - Explicație Detaliată

### Hit Probability (`sensor_model/hit`)

Controlează cât de repede un voxel devine OCCUPIED:

```
Current = 0.12 (FREE)
Frame 1: detectat obstacol → 0.12 + 0.7 = 0.82
Frame 2: detectat obstacol → 0.82 + (0.97-0.82)*0.7 = 0.93
Frame 3: detectat obstacol → 0.93 + (0.97-0.93)*0.7 = 0.96 → OCCUPIED
```

**Valori:**
- `0.7` (default) - 2-3 frame-uri pentru a deveni OCCUPIED
- `0.9` - 1-2 frame-uri (mai agresiv, mai mult noise)
- `0.5` - 4-5 frame-uri (mai conservator, mai puțin noise)

### Miss Probability (`sensor_model/miss`)

Controlează cât de repede un voxel devine FREE:

```
Current = 0.97 (OCCUPIED)
Frame 1: camera vede liber → 0.97 - 0.4 = 0.57
Frame 2: camera vede liber → 0.57 - 0.4 = 0.17
Frame 3: camera vede liber → 0.17 - 0.4 = -0.23 → clamped la 0.12 → FREE
```

**Valori:**
- `0.4` (default) - 2-3 frame-uri pentru clearing
- `0.6` - 1-2 frame-uri (clearing rapid)
- `0.2` - 5-6 frame-uri (clearing lent, mai stabil)

### Min/Max Clamping

```yaml
sensor_model/min: 0.12   # Orice sub 0.12 = FREE (șters din vizualizare)
sensor_model/max: 0.97   # Orice peste 0.97 = OCCUPIED (afișat în RViz)
```

**Zona gri (0.12 - 0.97):** "UNKNOWN" - nedecis încă

## Troubleshooting

### Problemă: Obiectele vechi NU dispar

**Cauze posibile:**

1. **Free space NU e activat**
```bash
# Verifică config
grep "publish_free_space" src/bringup/arm_system_bringup/config/octomap_server.yaml
# Ar trebui: publish_free_space: true

grep "publish_free_space" src/planning/arm_moveit_config/config/sensors_3d.yaml
# Ar trebui: publish_free_space: true
```

2. **Camera nu vede poziția veche**
```bash
# Verifică FOV camera și poziție
ros2 topic echo /camera/depth/points --field width
ros2 topic echo /camera/depth/points --field height

# Asigură-te că camera vede PRIN zona unde era obiectul
```

3. **Update rate prea mic**
```bash
# Verifică update rate efectiv
ros2 topic hz /octomap_binary
# Ar trebui: ~5 Hz

ros2 topic hz /planning_scene
# Ar trebui: ~5 Hz
```

**Soluție:**
```bash
# Rebuild cu configurări corecte
colcon build --packages-select arm_perception arm_moveit_config arm_system_bringup

# Source
source install/setup.bash

# Restart sistemul
```

### Problemă: Obiectele dispar prea repede (false negatives)

**Cauze:**
- `sensor_model/miss` prea mare
- `max_update_rate` prea mare
- Camera oscilează (motion blur)

**Soluție:**
```yaml
# Reduce decay rate
sensor_model/miss: 0.3    # de la 0.4
max_update_rate: 3.0      # de la 5.0
```

### Problemă: Obiectele persistă prea mult (clearing lent)

**Cauze:**
- `sensor_model/miss` prea mic
- `max_update_rate` prea mic
- Camera nu vede zona

**Soluție:**
```yaml
# Crește decay rate
sensor_model/miss: 0.5    # de la 0.4
max_update_rate: 10.0     # de la 5.0
```

### Problemă: CPU usage prea mare

**Soluție:**
```yaml
# În sensors_3d.yaml
point_subsample: 2        # de la 1 (reduce density)
max_update_rate: 2.0      # de la 5.0 (updates mai rare)

# În octomap_server.yaml
resolution: 0.1           # de la 0.05 (voxeli mai mari)
```

## Verificare Rapidă - Checklist

✅ **Config corecte:**
```bash
# 1. sensors_3d.yaml
grep "publish_free_space: true" src/planning/arm_moveit_config/config/sensors_3d.yaml

# 2. octomap_server.yaml
grep "publish_free_space: true" src/bringup/arm_system_bringup/config/octomap_server.yaml

# 3. Update rate
grep "max_update_rate: 5.0" src/planning/arm_moveit_config/config/sensors_3d.yaml
```

✅ **Sistem pornit corect:**
```bash
# Verifică noduri
ros2 node list | grep -E "(octomap|planning_scene)"
# Ar trebui să vezi:
# - /octomap_server
# - /planning_scene_updater

# Verifică topic-uri
ros2 topic list | grep -E "(octomap|planning_scene)"
# Ar trebui să vezi:
# - /octomap_binary
# - /planning_scene
```

✅ **Date curge corect:**
```bash
# Verifică rate-uri
ros2 topic hz /camera/depth/points    # ~30 Hz
ros2 topic hz /octomap_binary         # ~5 Hz
ros2 topic hz /planning_scene         # ~5 Hz
```

## Arhitectură - Data Flow

```
┌──────────────┐
│ Depth Camera │ 30 Hz
│   /camera/   │
│ depth/points │
└──────┬───────┘
       │
       ▼
┌──────────────────────┐
│  OctoMap Server      │ 5 Hz
│  - Raycasting        │
│  - Probabilistic     │
│    updates           │
│  - FREE + OCCUPIED   │
└──────┬───────────────┘
       │
       ├─→ /octomap_binary (occupied voxels)
       ├─→ /free_cells_vis_array (free voxels - debug)
       └─→ /occupied_cells_vis_array (occupied voxels - viz)
       │
       ▼
┌──────────────────────┐
│ Planning Scene       │ 5 Hz
│ Updater              │
│ - Differential       │
│   updates            │
│ - FREE clears old    │
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│ MoveIt move_group    │
│ Planning Scene       │
│ - Collision checking │
│ - Motion planning    │
└──────────────────────┘
```

## Referințe

- [OctoMap Probabilistic Model](https://octomap.github.io/octomap/doc/structoctomap_1_1OcTreeNode.html)
- [MoveIt Planning Scene Monitor](https://moveit.picknik.ai/main/api/html/classplanning__scene__monitor_1_1PlanningSceneMonitor.html)
- [Sensor Model Tuning](https://octomap.github.io/octomap/doc/OcTreeNode_8h_source.html)

## Exemple Practice

### Exemplu 1: Scenă cu Obiecte Mobile

```bash
# Setup: Cube care se mișcă în fața camerei

# În Gazebo: Insert → Box → Plasează în fața camerei
# Așteaptă 2s să apară în OctoMap
# Mută cubul cu mouse-ul

# Observă în RViz:
# - Vechea poziție devine transparentă în ~0.4s
# - Complet ștearsă în ~0.6s
# - Noua poziție apare în ~0.2s
```

### Exemplu 2: Testare cu Roboți Mobili

```bash
# Scenario: Robot trecând prin câmpul vizual

# Camera vede:
# Frame 1-5: Robot la X=1.0m → OCCUPIED
# Frame 6: Robot s-a mutat la X=2.0m
# Frame 6-8: Camera vede X=1.0m liber → FREE
# Frame 9: X=1.0m șters, X=2.0m OCCUPIED
```

## Contribuții

Pentru îmbunătățiri:
- Ajustează `sensor_model/*` în [octomap_server.yaml](src/bringup/arm_system_bringup/config/octomap_server.yaml)
- Modifică `max_update_rate` în [sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml)
- Test pe scenarii diverse (obiecte rapide, lente, statice)
