# Perception Tests Package

Quick and easy object grasp detection using OpenCV.

## What This Package Does

Detects objects in camera feed and computes optimal grasp points using computer vision.

**Key Features:**
- Multiple grasp strategies (center, edge, corner, handle)
- Works with webcam or RGB-D camera
- Real-time detection and visualization
- ROS 2 integration for robot control

## Quick Start

### Ultra-Simple Test (No ROS)
```bash
cd ~/Documents/GitHub/ldr-humanoid-arm-system
./test_grasp_simple.sh
```

Opens an OpenCV window showing detected objects and grasp points. Press 'q' to quit.

### ROS 2 Webcam Test
```bash
cd ~/Documents/GitHub/ldr-humanoid-arm-system
./test_webcam_grasp.sh
```

Publishes grasp poses to ROS 2 topics.

## Documentation

- **[GRASP_DETECTION_SUMMARY.md](../../../GRASP_DETECTION_SUMMARY.md)** - Complete overview of all versions
- **[WEBCAM_GRASP_QUICKSTART.md](../../../WEBCAM_GRASP_QUICKSTART.md)** - Webcam-specific guide
- **[README_GRASP_DETECTION.md](README_GRASP_DETECTION.md)** - RGB-D camera guide

## Scripts

| Script | Description |
|--------|-------------|
| `webcam_grasp_standalone.py` | Standalone OpenCV window (no ROS) |
| `webcam_grasp_detector.py` | ROS 2 node using webcam |
| `opencv_grasp_detector.py` | ROS 2 node using RGB-D camera |

## Visualization

Detected grasp points are color-coded:
- **Cyan** - Center grasp (most reliable)
- **Magenta** - Edge grasp (for elongated objects)
- **Yellow** - Corner grasp (for boxes)
- **Orange** - Handle grasp (for mugs, bags)

## Requirements

- Python 3
- OpenCV (cv2) - installed ✓
- NumPy - installed ✓
- ROS 2 Jazzy - for ROS versions only

## Examples

### Example 1: Detect Mug Handle
```bash
./test_grasp_simple.sh
# Place mug in front of camera
# → Orange arrow appears at handle
```

### Example 2: Get Grasp Pose for Robot
```bash
ros2 run perception_tests webcam_grasp_detector.py

# In another terminal:
ros2 topic echo /grasp_detector/best_grasp
# → Shows 3D pose of best grasp point
```

## Integration with MoveIt

```python
import rclpy
from geometry_msgs.msg import PoseStamped

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('executor')
        self.create_subscription(PoseStamped, '/grasp_detector/best_grasp',
                                self.execute_grasp, 10)

    def execute_grasp(self, pose_msg):
        # Send to MoveIt arm controller
        # arm.set_pose_target(pose_msg.pose)
        # arm.go()
        pass
```

## Package Structure

```
perception_tests/
├── scripts/
│   ├── webcam_grasp_standalone.py    # No ROS, direct window
│   ├── webcam_grasp_detector.py      # ROS + webcam
│   └── opencv_grasp_detector.py      # ROS + RGB-D
├── launch/
│   └── opencv_grasp_demo.launch.py
├── config/
└── README.md (this file)
```

## Building

```bash
cd ~/Documents/GitHub/ldr-humanoid-arm-system
colcon build --packages-select perception_tests
source install/setup.bash
```

## Troubleshooting

**No detections?**
- Improve lighting
- Use plain background
- Adjust `min_object_area` parameter

**Webcam not opening?**
```bash
ls -l /dev/video*  # Check available cameras
```

**Too sensitive?**
- Press '+' key in standalone mode
- Or increase `min_object_area` parameter

## Next Steps

1. ✅ Test with `./test_grasp_simple.sh`
2. Place various objects (mug, bottle, box) in front of camera
3. Observe different grasp strategies
4. Integrate with arm controller using `/grasp_detector/best_grasp` topic

---

**For more details, see [GRASP_DETECTION_SUMMARY.md](../../../GRASP_DETECTION_SUMMARY.md)**
