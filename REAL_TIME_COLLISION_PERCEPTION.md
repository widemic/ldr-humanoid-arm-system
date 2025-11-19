# Percepție Coliziune în Timp Real cu MoveIt

Acest ghid explică cum să folosești sistemul de percepție 3D actualizat pentru detecția coliziunilor în timp real în MoveIt.

## Prezentare Generală

Sistemul combină:
- **OctoMap** - Hartă 3D ocupațională din point cloud-uri
- **Planning Scene Monitor** - Actualizează scena de planificare MoveIt în timp real
- **Depth Camera** - Senzor RGBD pentru percepția mediului

## Modificări Efectuate

### 1. Senzori 3D Optimizați ([sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml))

```yaml
max_update_rate: 5.0         # Crescut de la 1.0 la 5.0 Hz
padding_offset: 0.05         # 5cm padding pentru siguranță
point_subsample: 1           # Folosește toate punctele pentru acuratețe
queue_size: 10              # Crescut pentru rate mai mari
```

**Îmbunătățiri:**
- Actualizări de 5x mai rapide (5 Hz vs 1 Hz)
- Mai mult padding pentru mișcări sigure
- Queue mai mare pentru procesare fără pierderi

### 2. Planning Scene Updater Nou

Creat nod Python dedicat: [planning_scene_updater.py](src/perception/arm_perception/scripts/planning_scene_updater.py)

**Funcționalitate:**
- Subscribe la `/octomap_binary` sau `/octomap_full`
- Publică la `/planning_scene` pentru MoveIt
- Rate configurabil (default 5 Hz)
- Actualizări diferențiale pentru performanță

### 3. Perception Launch Actualizat ([perception.launch.py](src/perception/arm_perception/launch/perception.launch.py))

**Componente lansate:**
1. Perception node (detecție obiecte geometrice)
2. OctoMap server (mapping 3D)
3. Planning scene updater (actualizări MoveIt)

## Cum să Folosești Sistemul

### Opțiunea 1: Sistem Complet cu Percepție Activă

```bash
# Terminal 1: Gazebo + MoveIt
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Terminal 2: Percepție în timp real (după 20s)
ros2 launch arm_perception perception.launch.py
```

### Opțiunea 2: Sistem Integrat cu OctoMap Vizualizare

```bash
# Tot-în-unu cu OctoMap și coliziune în RViz
ros2 launch arm_system_bringup moveit_gazebo_with_octomap.launch.py
```

### Opțiunea 3: Modulară (Control Maxim)

```bash
# Terminal 1: Gazebo
ros2 launch arm_gazebo arm_world.launch.py

# Terminal 2: Robot + Controllers (după 5s)
ros2 launch arm_gazebo spawn_arm.launch.py

# Terminal 3: MoveIt (după 15s)
ros2 launch arm_moveit_config demo.launch.py

# Terminal 4: Percepție (după 20s)
ros2 launch arm_perception perception.launch.py
```

## Parametri de Configurare

### Perception Launch

```bash
# Activează/dezactivează OctoMap
ros2 launch arm_perception perception.launch.py enable_octomap:=true

# Schimbă topicul point cloud
ros2 launch arm_perception perception.launch.py pointcloud_topic:=/camera/depth/points

# Schimbă frame-ul de referință
ros2 launch arm_perception perception.launch.py frame_id:=base_fixture_link

# Folosește timp simulare
ros2 launch arm_perception perception.launch.py use_sim_time:=true
```

### Planning Scene Updater

Parametri în [perception.launch.py](src/perception/arm_perception/launch/perception.launch.py) (linia 107):

```yaml
update_rate: 5.0          # Hz - frecvența actualizărilor
octomap_frame: "base_fixture_link"  # Frame de referință
```

## Verificare Funcționare

### 1. Verifică Topic-uri Active

```bash
# OctoMap este publicat?
ros2 topic hz /octomap_binary

# Planning scene este actualizată?
ros2 topic hz /planning_scene

# Point cloud este activ?
ros2 topic hz /camera/depth/points
```

### 2. Monitorizează Log-uri

```bash
# Planning scene updater
ros2 node info /planning_scene_updater

# Vezi actualizări în timp real
ros2 topic echo /planning_scene --field is_diff
```

### 3. Verificare în RViz

1. Deschide RViz cu config MoveIt
2. Adaugă display **MarkerArray** → topic `/occupied_cells_vis_array`
3. Adaugă display **PlanningScene** → topic `/planning_scene`
4. Activează **Scene Geometry**
5. Vei vedea OctoMap-ul colorat și coliziunile în timp real

## Performanță și Optimizare

### Rate de Actualizare Recomandate

| Componentă | Rate (Hz) | Justificare |
|-----------|-----------|-------------|
| Point Cloud | 30 | Hardware camera |
| OctoMap Update | 5 | Echilibru CPU/acuratețe |
| Planning Scene | 5 | Sincronizat cu OctoMap |
| Joint States | 10 | Rapid pentru planning |

### Ajustări pentru Performanță

**Dacă sistemul este lent:**
```yaml
# În sensors_3d.yaml
max_update_rate: 2.0      # Reduce de la 5.0
point_subsample: 2        # Subsample point cloud

# În octomap_server.yaml
resolution: 0.1           # Crescut de la 0.05 (voxeli mai mari)
```

**Pentru acuratețe maximă:**
```yaml
# În sensors_3d.yaml
max_update_rate: 10.0     # Crescut de la 5.0
point_subsample: 1        # Toate punctele

# În octomap_server.yaml
resolution: 0.02          # Redus de la 0.05 (voxeli mai mici)
```

## Arhitectură Sistem

```
┌─────────────────┐
│  Depth Camera   │
│ /camera/depth/  │
│     points      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ OctoMap Server  │
│ /octomap_binary │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Planning Scene │◄─── Used by MoveIt Planning
│     Updater     │
│ /planning_scene │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  MoveIt Move    │
│      Group      │
│  (Collision-    │
│   aware plan)   │
└─────────────────┘
```

## Fișiere Modificate

1. [src/planning/arm_moveit_config/config/sensors_3d.yaml](src/planning/arm_moveit_config/config/sensors_3d.yaml)
   - Rate crescut la 5 Hz
   - Padding optimizat
   - Queue size mărit

2. [src/perception/arm_perception/launch/perception.launch.py](src/perception/arm_perception/launch/perception.launch.py)
   - Adăugat OctoMap server launch
   - Adăugat planning scene updater
   - Parametri configurabili

3. [src/perception/arm_perception/scripts/planning_scene_updater.py](src/perception/arm_perception/scripts/planning_scene_updater.py)
   - Nod nou pentru actualizări MoveIt
   - Subscribe la OctoMap
   - Publică differential planning scene

4. [src/perception/arm_perception/CMakeLists.txt](src/perception/arm_perception/CMakeLists.txt)
   - Instalează planning_scene_updater.py

5. [src/perception/arm_perception/package.xml](src/perception/arm_perception/package.xml)
   - Adăugate dependențe: moveit_msgs, octomap_msgs

## Troubleshooting

### OctoMap nu apare în RViz

**Problem:** Nu vezi voxeli în RViz

**Soluție:**
```bash
# Verifică că camera publică date
ros2 topic hz /camera/depth/points

# Verifică că OctoMap server rulează
ros2 node list | grep octomap

# Adaugă MarkerArray display cu topic corect
/occupied_cells_vis_array
```

### Planning eșuează cu coliziuni

**Problem:** MoveIt spune că există coliziune când nu ar trebui

**Soluție:**
```yaml
# În sensors_3d.yaml - reduce padding-ul
padding_offset: 0.02      # De la 0.05
padding_scale: 1.0
```

### Actualizări prea lente

**Problem:** Scena de planificare nu se actualizează rapid

**Soluție:**
```bash
# Verifică rate-ul real
ros2 topic hz /planning_scene

# Crește update_rate în perception.launch.py
update_rate: 10.0  # De la 5.0
```

### CPU Usage Mare

**Problem:** Sistemul consumă prea mult CPU

**Soluție:**
```yaml
# Reduce rezoluția OctoMap
resolution: 0.1     # În octomap_server.yaml

# Subsample point cloud
point_subsample: 2  # În sensors_3d.yaml

# Reduce rate
max_update_rate: 2.0
```

## Referințe

- [MoveIt Perception](https://moveit.picknik.ai/main/doc/how_to_guides/perception_pipeline/perception_pipeline_tutorial.html)
- [OctoMap ROS2](http://wiki.ros.org/octomap)
- [Planning Scene Monitor](https://moveit.picknik.ai/main/api/html/classplanning__scene__monitor_1_1PlanningSceneMonitor.html)

## Suport

Pentru probleme sau întrebări:
- GitHub Issues: https://github.com/widemic/ldr-humanoid-arm-system
- Documentație proiect: [CLAUDE.md](CLAUDE.md)
