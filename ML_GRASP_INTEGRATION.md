# ML Grasp Detection Integration Guide

How to integrate learned grasp detection models (GraspNet, Contact-GraspNet, etc.) into the system.

## Current Status

✅ **Implemented:**
- Robust object detection with multi-method approach
- Object tracking and stability filtering
- Shape and color feature extraction
- ML grasp detector framework (placeholder model)
- Full ROS 2 integration

🔨 **To Be Added:**
- Real ML model integration (GraspNet, Contact-GraspNet, etc.)
- Model weight downloads
- GPU acceleration

## Architecture

```
                  ┌─────────────────────────────┐
                  │   Webcam / RGB-D Camera     │
                  └──────────┬──────────────────┘
                             │
                             ▼
                  ┌─────────────────────────────┐
                  │  Robust Object Detection    │
                  │  - Multi-method detection   │
                  │  - Temporal tracking        │
                  │  - Feature extraction       │
                  └──────────┬──────────────────┘
                             │
                             ▼
                  ┌─────────────────────────────┐
                  │   ML Grasp Prediction       │
                  │  - GraspNet / Contact-GN    │
                  │  - 6-DOF grasp poses        │
                  │  - Quality scores           │
                  └──────────┬──────────────────┘
                             │
                             ▼
                  ┌─────────────────────────────┐
                  │  Grasp Ranking & Selection  │
                  │  - Filter by quality        │
                  │  - Gripper constraints      │
                  │  - Collision checking       │
                  └──────────┬──────────────────┘
                             │
                             ▼
                  ┌─────────────────────────────┐
                  │    MoveIt / MTC Execution   │
                  │  - Motion planning          │
                  │  - Grasp execution          │
                  └─────────────────────────────┘
```

## Two-Stage Detection System

### Stage 1: Robust Object Detection

[robust_object_detector.py](src/perception/perception_tests/scripts/robust_object_detector.py) provides reliable object detection before grasp planning.

**Features:**
- **Multiple detection methods**: Adaptive threshold, Canny edges, color segmentation
- **Temporal tracking**: Objects must appear for multiple frames to be "stable"
- **Rich features**: Shape (circularity, aspect ratio), color, size
- **Outlier rejection**: Filters false positives

**Run:**
```bash
ros2 run perception_tests robust_object_detector.py
```

**Topics:**
- `/object_detector/visualization` - Annotated image
- `/object_detector/objects` - JSON with detected objects and features
- `/object_detector/markers` - RViz markers

**Key Parameters:**
```bash
ros2 run perception_tests robust_object_detector.py --ros-args \
  -p detection_method:=multi \           # 'adaptive', 'canny', 'multi'
  -p stability_threshold:=5 \            # Frames before object is stable
  -p min_object_area:=2000
```

### Stage 2: ML Grasp Prediction

[ml_grasp_detector.py](src/perception/perception_tests/scripts/ml_grasp_detector.py) predicts optimal grasps using learned models.

**Current:** Placeholder (geometric heuristics)
**Goal:** Integrate GraspNet / Contact-GraspNet

**Run:**
```bash
ros2 run perception_tests ml_grasp_detector.py
```

**Topics:**
- `/ml_grasp/visualization` - Grasps visualized on image
- `/ml_grasp/predictions` - JSON with all grasp candidates
- `/ml_grasp/best_grasp` - Best grasp (PoseStamped)
- `/ml_grasp/markers` - RViz grasp arrows

## Integrating Real ML Models

### Option 1: GraspNet-1Billion

**Best for:** Parallel-jaw grippers, large object dataset training

**Installation:**
```bash
# Install dependencies
pip install torch torchvision
pip install open3d
pip install graspnetAPI

# Download GraspNet-1Billion weights
mkdir -p ~/grasp_models
cd ~/grasp_models
wget https://graspnet.net/models/checkpoint-rs.tar  # RealSense trained
# or
wget https://graspnet.net/models/checkpoint-kn.tar  # Kinect trained
```

**Integration Code:**

Edit [ml_grasp_detector.py](src/perception/perception_tests/scripts/ml_grasp_detector.py):

```python
# At top of file:
import torch
import graspnetAPI
from graspnetAPI import GraspGroup

# In load_graspnet_model():
def load_graspnet_model(self):
    """Load GraspNet model."""
    try:
        from graspnetAPI import GraspNetAPI

        checkpoint_path = os.path.expanduser('~/grasp_models/checkpoint-rs.tar')

        self.get_logger().info(f'Loading GraspNet from {checkpoint_path}')

        # Initialize model
        model = GraspNetAPI(
            checkpoint_path=checkpoint_path,
            camera='realsense',  # or 'kinect'
            num_point=20000,
            num_view=300,
            collision_thresh=0.01,
            voxel_size=0.01
        )

        self.get_logger().info('GraspNet loaded successfully')
        return model

    except Exception as e:
        self.get_logger().error(f'Failed to load GraspNet: {e}')
        return None

# In predict_grasps_with_model():
def predict_grasps_with_model(self, rgb, depth, obj_mask):
    """Run GraspNet inference."""

    # Convert to point cloud
    points = self.depth_to_point_cloud(depth, rgb)

    # Filter to object region
    obj_points = points[obj_mask.flatten() > 0]

    # Run GraspNet
    grasp_group = self.model.get_grasp(
        point_cloud=obj_points,
        num_grasp=self.max_grasps
    )

    # Convert to GraspPrediction format
    grasps = []
    for i in range(len(grasp_group)):
        g = grasp_group[i]

        grasp = GraspPrediction()
        grasp.position = (g.translation[0], g.translation[1], g.translation[2])
        grasp.orientation = self.rotation_matrix_to_quaternion(g.rotation_matrix)
        grasp.width = g.width
        grasp.quality = g.score

        grasps.append(grasp)

    return grasps
```

**Run with GraspNet:**
```bash
ros2 run perception_tests ml_grasp_detector.py --ros-args \
  -p model_type:=graspnet \
  -p model_path:=~/grasp_models/checkpoint-rs.tar \
  -p min_grasp_quality:=0.7
```

### Option 2: Contact-GraspNet

**Best for:** Diverse object shapes, contact-based grasp quality

**Installation:**
```bash
# Clone Contact-GraspNet
cd ~/grasp_models
git clone https://github.com/NVlabs/contact_graspnet.git
cd contact_graspnet

# Install dependencies
pip install -r requirements.txt

# Download pretrained weights
bash checkpoints/download_checkpoints.sh
```

**Integration Code:**

```python
# In load_contact_graspnet_model():
def load_contact_graspnet_model(self):
    """Load Contact-GraspNet model."""
    import tensorflow as tf
    import sys
    sys.path.append(os.path.expanduser('~/grasp_models/contact_graspnet'))

    from contact_graspnet_pytorch import ContactGraspNet

    model_path = os.path.expanduser('~/grasp_models/contact_graspnet/checkpoints')

    model = ContactGraspNet(
        checkpoint_dir=model_path,
        k_candidates=self.max_grasps
    )

    self.get_logger().info('Contact-GraspNet loaded')
    return model

def predict_grasps_with_model(self, rgb, depth, obj_mask):
    """Run Contact-GraspNet."""

    # Prepare input
    segmap = obj_mask.astype(np.uint8)

    # Run inference
    pred_grasps_cam, scores, contact_pts = self.model.predict_scene_grasps(
        rgb_image=rgb,
        depth_image=depth,
        segmap=segmap,
        forward_passes=1
    )

    # Convert to GraspPrediction
    grasps = []
    for i in range(len(pred_grasps_cam)):
        grasp = GraspPrediction()

        # Extract pose
        grasp_pose = pred_grasps_cam[i]  # 4x4 matrix
        grasp.position = tuple(grasp_pose[:3, 3])
        grasp.orientation = self.rotation_matrix_to_quaternion(grasp_pose[:3, :3])
        grasp.quality = scores[i]
        grasp.width = 0.08  # Default gripper width

        grasps.append(grasp)

    return grasps
```

### Option 3: Custom YOLO + Heuristic Grasps

**Best for:** Quick prototyping with semantic understanding

**Installation:**
```bash
pip install ultralytics  # YOLOv8
```

**Integration:**

```python
from ultralytics import YOLO

# In __init__:
self.yolo_model = YOLO('yolov8n.pt')  # Nano model (fast)

# In detect_objects:
def detect_objects(self, image):
    """YOLO-based object detection."""

    results = self.yolo_model(image)

    objects = []
    for result in results:
        boxes = result.boxes

        for box in boxes:
            # Get bounding box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # Create mask
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            mask[int(y1):int(y2), int(x1):int(x2)] = 255

            # Center
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Class name
            class_name = result.names[int(box.cls)]

            objects.append((mask, (cx, cy), class_name))

    return objects
```

## Testing the ML Pipeline

### 1. Test Robust Detection First

```bash
# Terminal 1: Run detector
ros2 run perception_tests robust_object_detector.py

# Terminal 2: Monitor objects
ros2 topic echo /object_detector/objects

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /object_detector/visualization
```

**What to check:**
- Objects appear consistently (green = stable)
- False positives are filtered out
- Features are reasonable (circularity, color, etc.)

### 2. Test ML Grasp Detector (Placeholder)

```bash
# Run ML detector
ros2 run perception_tests ml_grasp_detector.py

# Check grasps
ros2 topic echo /ml_grasp/best_grasp

# Visualize
ros2 run rqt_image_view rqt_image_view /ml_grasp/visualization
```

**What to check:**
- Multiple grasp candidates generated
- Quality scores assigned
- Best grasp selected and published

### 3. Integrate Real Model

```bash
# After installing GraspNet (see above)
ros2 run perception_tests ml_grasp_detector.py --ros-args \
  -p model_type:=graspnet \
  -p model_path:=~/grasp_models/checkpoint-rs.tar \
  -p min_grasp_quality:=0.5
```

## Performance Optimization

### GPU Acceleration

Most ML models require GPU for real-time performance.

**Check CUDA availability:**
```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

**Enable GPU in model:**
```python
# In load_model:
if torch.cuda.is_available():
    self.model = self.model.cuda()
    self.get_logger().info('Using GPU acceleration')
else:
    self.get_logger().warn('GPU not available, using CPU (slower)')
```

### Model Optimization

1. **Use smaller models**: YOLOv8n instead of YOLOv8x
2. **Reduce point cloud size**: Downsample before inference
3. **Lower inference rate**: Run ML at 2-5 Hz, not 30 Hz
4. **Batch processing**: Process multiple candidates together

## Integration with MoveIt

Once grasps are detected, send them to MoveIt:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')

        # MoveIt interface
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")

        # Subscribe to ML grasps
        self.create_subscription(
            PoseStamped,
            '/ml_grasp/best_grasp',
            self.execute_grasp,
            10
        )

    def execute_grasp(self, grasp_pose: PoseStamped):
        """Execute detected grasp."""
        self.get_logger().info(f'Executing grasp at {grasp_pose.pose.position}')

        # Pre-grasp pose (approach from above)
        pre_grasp = grasp_pose
        pre_grasp.pose.position.z += 0.1  # 10cm above

        # Plan to pre-grasp
        self.arm.set_goal_state(pose_stamped_msg=pre_grasp)
        plan = self.arm.plan()

        if plan:
            self.arm.execute()

            # Move to grasp
            self.arm.set_goal_state(pose_stamped_msg=grasp_pose)
            plan = self.arm.plan()

            if plan:
                self.arm.execute()
                # Close gripper here
                self.get_logger().info('Grasp executed!')
```

## Troubleshooting

### Objects not detected reliably

**Solution:**
```bash
# Use multi-method detection
ros2 run perception_tests robust_object_detector.py --ros-args \
  -p detection_method:=multi \
  -p stability_threshold:=3
```

### ML model out of memory

**Solution:**
- Reduce point cloud size: `num_point=10000` instead of 20000
- Use smaller model variant
- Enable model pruning/quantization

### Grasps have low quality

**Solution:**
- Check camera calibration
- Verify depth data quality
- Lower `min_grasp_quality` threshold
- Retrain model on your specific objects

## Next Steps

1. **Choose a model**: GraspNet (general), Contact-GraspNet (contact-based), or YOLO+heuristics (fast)
2. **Install dependencies**: Follow installation instructions above
3. **Integrate model**: Modify `ml_grasp_detector.py` with model code
4. **Test thoroughly**: Use test objects, verify grasp quality
5. **Connect to robot**: Send grasps to MoveIt for execution

## References

- **GraspNet-1Billion**: [https://graspnet.net/](https://graspnet.net/)
- **Contact-GraspNet**: [https://github.com/NVlabs/contact_graspnet](https://github.com/NVlabs/contact_graspnet)
- **YOLOv8**: [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)
- **MoveIt 2**: [https://moveit.ros.org/](https://moveit.ros.org/)

---

**Current status:** Robust object detection ✅, ML framework ✅, Real model integration ⏳
