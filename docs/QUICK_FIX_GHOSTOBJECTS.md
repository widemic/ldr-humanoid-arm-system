# FIX RAPID: Eliminare "Fantome" (Ghost Objects)

## Problema

Când muți un obiect (ex: human model) în Gazebo:
- ❌ Poziția veche rămâne vizibilă ("fantomă")
- ❌ Collision objects nu urmăresc obiectul
- ❌ Bounding box nu reflectă forma corect

## Cauza

**OctoMap și Object Tracker rulează SIMULTAN și se bat între ei!**

- OctoMap: Clearing lent (0.4-0.6s) → creează "fantome"
- Object Tracker: Clearing instant → dar e suprascris de OctoMap

## Soluția: Folosește DOAR Object Tracking

### Pasul 1: OPREȘTE Tot Ce Rulează

```bash
# În toate terminalele cu perception, apasă Ctrl+C
```

### Pasul 2: Lansează DOAR Object Tracking

```bash
# În directorul proiectului
cd /home/andrei/ros2_ws/ldr-humanoid-arm-system

# Source workspace
source install/setup.bash

# Lansează tracking FĂRĂ OctoMap
./launch_tracking_only.sh
```

SAU manual:

```bash
source install/setup.bash
ros2 launch arm_perception perception.launch.py \
    enable_octomap:=false \
    enable_object_tracking:=true
```

### Pasul 3: Verificare în RViz

**Adaugă displays:**

1. **MarkerArray**
   - Topic: `/tracked_objects`
   - Vei vedea: Bounding boxes verzi care URMĂRESC obiectele

2. **PlanningScene**
   - Enable: Scene Geometry
   - Vei vedea: Collision objects actualizate în timp real

3. **PointCloud2** (opțional - debugging)
   - Topic: `/camera/depth/points`
   - Vei vedea: Point cloud-ul raw

### Pasul 4: Test

```bash
# În Gazebo:
# 1. Plasează/inserează human model în fața camerei
# 2. Mută human model cu mouse-ul
# 3. În RViz: Marker-ul verde ar trebui să URMĂREASCĂ modelul
# 4. NU ar trebui să vezi "fantome"!
```

## Parametri Optimizați (pentru Human Models)

Am setat automat parametrii optimi pentru obiecte complexe:

```yaml
min_cluster_size: 100       # Filtrează noise mic
max_cluster_size: 10000     # Permite obiecte mari (humans)
cluster_tolerance: 0.15     # 15cm - bun pentru forme complexe
max_tracking_distance: 0.5  # 50cm - permite mișcări rapide
min_object_height: 0.05     # 5cm - filtrează ground plane
```

## Troubleshooting

### Problema: NU văd markeri în RViz

**Verifică că tracker-ul rulează:**
```bash
ros2 node list | grep dynamic_object_tracker
# Ar trebui să vezi: /dynamic_object_tracker
```

**Verifică logs:**
```bash
ros2 topic echo /rosout | grep "dynamic_object_tracker"
# Ar trebui să vezi: "Frame X: YYYY valid points"
```

**Dacă vezi "Frame X: 0 valid points":**
```bash
# Toate punctele sunt filtrate!
# Reduce min_object_height
ros2 param set /dynamic_object_tracker min_object_height 0.0
```

### Problema: Obiectele sunt prea mici/mari

**Ajustează bounding box:**
```bash
# Obiecte mai mari (human model e sub-detectat)
ros2 param set /dynamic_object_tracker cluster_tolerance 0.2

# Obiecte mai mici (prea mult clustering)
ros2 param set /dynamic_object_tracker cluster_tolerance 0.1
```

### Problema: "Fantome" ÎNCĂ Persistă

**Verifică că OctoMap NU rulează:**
```bash
ros2 node list | grep octomap
# Ar trebui: FĂRĂ output!

# Dacă vezi /octomap_server:
ros2 node kill /octomap_server
ros2 node kill /planning_scene_updater
```

### Problema: Obiectul nu e detectat deloc

**Verifică că e în fața camerei:**
```bash
# Vezi point cloud în RViz
# Add → PointCloud2 → /camera/depth/points

# Ar trebui să vezi puncte acolo unde e obiectul
```

**Verifică dimensiuni cluster:**
```bash
# Obiect foarte mic
ros2 param set /dynamic_object_tracker min_cluster_size 50

# Obiect foarte mare
ros2 param set /dynamic_object_tracker max_cluster_size 20000
```

## De Ce Funcționează Acum?

### ÎNAINTE (cu OctoMap + Tracker):
```
Frame 1: Object la poziția A
  - OctoMap: Marchează A ca OCCUPIED
  - Tracker: Adaugă collision_object_0 la A

Frame 10: Object mutat la poziția B
  - Tracker: REMOVE collision_object_0 la A ✓
  - Tracker: ADD collision_object_0 la B ✓
  - OctoMap: Încă are voxeli la A ❌ (clearing lent)
  - OctoMap: Publică voxeli la A peste tracker ❌

Rezultat: Vezi "fantomă" la A + obiect la B
```

### ACUM (DOAR Tracker):
```
Frame 1: Object la poziția A
  - Tracker: ADD collision_object_0 la A ✓

Frame 10: Object mutat la poziția B
  - Tracker: REMOVE collision_object_0 la A ✓
  - Tracker: ADD collision_object_0 la B ✓
  - (NU mai e OctoMap să suprascrie!)

Rezultat: Vezi DOAR obiect la B (fără fantomă!) ✅
```

## Comenzi Utile

```bash
# Verifică rate actualizări
ros2 topic hz /planning_scene

# Vezi collision objects
ros2 topic echo /planning_scene --field world.collision_objects[0]

# Monitorizează tracking logs
ros2 topic echo /rosout --field msg | grep -i "Object.*moved"

# Lista parametri tracker
ros2 param list /dynamic_object_tracker

# Schimbă parametri în timp real (fără restart)
ros2 param set /dynamic_object_tracker cluster_tolerance 0.2
```

## Limitări Curente

1. **Bounding Box Simple (AABB)**
   - Folosim Axis-Aligned Bounding Box
   - Pentru forme complexe (human), box-ul poate fi mai mare decât obiectul
   - Îmbunătățire viitoare: Oriented Bounding Box sau mesh shapes

2. **Nu Detectează Forma Exactă**
   - Creăm doar un cub simplu
   - Suficient pentru collision detection
   - Îmbunătățire viitoare: Shape recognition (detectează dacă e human/box/cylinder)

3. **Ground Plane Manual Filter**
   - Filtrăm puncte sub 5cm înălțime
   - Poate elimina obiecte foarte joase
   - Ajustează `min_object_height` dacă e nevoie

## Când Să Folosești OctoMap vs Tracker

| Use Case | OctoMap | Object Tracker | Recomandat |
|----------|---------|----------------|------------|
| **Obiecte mobile** | ❌ Fantome | ✅ Instant | **Tracker** ✨ |
| **Scene statice** | ✅ Detaliat | 🟡 Box simplu | OctoMap |
| **CPU limitat** | 🟡 Mediu | ✅ Scăzut | **Tracker** |
| **Forme complexe** | ✅ Voxeli | 🟡 Bounding box | OctoMap |
| **Tracking obiecte** | ❌ Nu trackuiește | ✅ ID-uri | **Tracker** ✨ |

**Pentru cazul tău (human model în mișcare): Folosește DOAR Object Tracker!** ⭐

---

**Creat:** 2025-11-19
**Update:** Fix pentru "ghost objects" cu human models
