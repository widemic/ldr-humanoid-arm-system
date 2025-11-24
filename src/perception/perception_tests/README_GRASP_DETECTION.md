# OpenCV Grasp Detection

Quick and easy object grasping position detection using OpenCV and RGB-D camera data.

## Overview

The `opencv_grasp_detector.py` node provides **single-file, lightweight object detection and grasp point estimation** using classical computer vision techniques. It detects objects, estimates their position and orientation, and suggests optimal grasp points.

### Features

- **Multiple grasp strategies**:
  - Center grasp (symmetric objects)
  - Edge grasp (elongated objects)
  - Corner grasp (rectangular objects)
  - Handle detection (concave regions)

- **3D pose estimation** using RGB-D data
- **Real-time visualization** with annotated images
- **RViz markers** for 3D visualization
- **Extensible** - ready for ML model integration (YOLO, etc.)

## Quick Start

### 1. Build the Package

```bash
cd ~/Documents/GitHub/ldr-humanoid-arm-system
colcon build --packages-select perception_tests
source install/setup.bash
```

### 2. Launch with Camera

If you have a camera running (e.g., RealSense, external camera):

```bash
# Terminal 1: Launch your camera
ros2 launch perception_tests external_camera_node.py

# Terminal 2: Launch grasp detector
ros2 launch perception_tests opencv_grasp_demo.launch.py
```

Or launch everything together:

```bash
ros2 launch perception_tests opencv_grasp_demo.launch.py use_camera:=true
```

### 3. View Results

**Check detected grasps:**
```bash
# View best grasp pose
ros2 topic echo /grasp_detector/best_grasp

# View visualization
ros2 run rqt_image_view rqt_image_view /grasp_detector/visualization
```

**RViz visualization:**
```bash
rviz2
# Add marker display for topic: /grasp_detector/markers
# Add image display for topic: /grasp_detector/visualization
```

## How It Works

### Detection Pipeline

```
RGB Image → Grayscale → Adaptive Threshold → Morphology
         ↓
    Find Contours → Filter by Area → Fit Bounding Box
         ↓
    Detect Object Type (circular, elongated, square, irregular)
         ↓
    Compute Grasp Points (center, edge, corner, handle)
         ↓
    Project to 3D using Depth → Publish Best Grasp
```

### Grasp Strategies

1. **Center Grasp** (confidence: 0.8)
   - Most robust, works for symmetric objects
   - Grasps at object centroid
   - Always available

2. **Edge Grasp** (confidence: 0.7)
   - For elongated objects (aspect ratio > 3)
   - Grasps along long edges perpendicular to main axis
   - Good for bottles, tools, etc.

3. **Corner Grasp** (confidence: 0.6)
   - For rectangular/square objects
   - Grasps at bounding box corners
   - Useful for boxes, books, etc.

4. **Handle Detection** (confidence: 0.5-0.9)
   - Uses convexity defects to find concave regions
   - Detects handles, loops, recesses
   - Good for mugs, bags with handles

### Object Classification

Simple shape-based classification:
- **Circular**: circularity > 0.8
- **Elongated**: aspect ratio > 3.0
- **Square**: aspect ratio < 1.5, circularity > 0.6
- **Irregular**: everything else

## Configuration

### Parameters

```bash
ros2 run perception_tests opencv_grasp_detector.py --ros-args \
  -p min_object_area:=1000 \
  -p max_object_area:=100000 \
  -p depth_min:=0.3 \
  -p depth_max:=2.0 \
  -p visualize:=true \
  -p camera_frame:=camera_color_optical_frame
```

**Parameters:**
- `min_object_area` (int, default: 1000): Minimum object size in pixels
- `max_object_area` (int, default: 100000): Maximum object size in pixels
- `depth_min` (float, default: 0.3): Minimum valid depth in meters
- `depth_max` (float, default: 2.0): Maximum valid depth in meters
- `visualize` (bool, default: true): Enable visualization publishing
- `camera_frame` (str, default: camera_color_optical_frame): Camera frame ID

### Topics

**Subscribed:**
- `/camera/color/image_raw` (sensor_msgs/Image): RGB image
- `/camera/depth/image_raw` (sensor_msgs/Image): Depth image (aligned to RGB)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

**Published:**
- `/grasp_detector/best_grasp` (geometry_msgs/PoseStamped): Best grasp pose
- `/grasp_detector/visualization` (sensor_msgs/Image): Annotated image
- `/grasp_detector/markers` (visualization_msgs/MarkerArray): RViz markers

## Visualization

The visualization image shows:
- **Green contours**: Detected objects
- **Blue boxes**: Oriented bounding boxes
- **Red dots**: Object centers
- **Colored circles with arrows**: Grasp points
  - Cyan (center grasp)
  - Magenta (edge grasp)
  - Yellow (corner grasp)
  - Orange (handle grasp)
- **Numbers**: Confidence values (0.0 to 1.0)

## Integration with MoveIt

Use the detected grasp pose with MoveIt Task Constructor:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages

class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')
        self.grasp_sub = self.create_subscription(
            PoseStamped, '/grasp_detector/best_grasp',
            self.grasp_callback, 10)

        self.task = core.Task("pick_detected_object")
        self.task.loadRobotModel()
        # ... setup task stages

    def grasp_callback(self, msg: PoseStamped):
        # Use msg.pose as target for MTC grasp stage
        self.execute_grasp(msg.pose)
```

## Extending with ML Models

The architecture is designed to integrate with deep learning models:

### Adding YOLO Object Detection

```python
# In opencv_grasp_detector.py, add YOLO model:
import cv2.dnn

class OpenCVGraspDetector(Node):
    def __init__(self):
        # ... existing code ...

        # Load YOLO
        self.yolo_net = cv2.dnn.readNet('yolov4.weights', 'yolov4.cfg')
        self.yolo_classes = open('coco.names').read().strip().split('\n')

    def detect_objects(self, image):
        # Option 1: Use YOLO for initial detection
        yolo_detections = self.detect_with_yolo(image)

        # Option 2: Use contours as fallback
        contour_detections = self.detect_with_contours(image)

        # Combine both methods
        return yolo_detections + contour_detections

    def detect_with_yolo(self, image):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True)
        self.yolo_net.setInput(blob)
        outputs = self.yolo_net.forward(self.yolo_net.getUnconnectedOutLayersNames())

        # Process YOLO outputs to create DetectedObject instances
        # ...
```

### Adding Bag/Deformable Object Detection

For complex objects like bags, you can add specialized detectors:

```python
def detect_bag_grasp_points(self, obj: DetectedObject, image: np.ndarray):
    """
    Specialized grasp detection for bags and deformable objects.
    Looks for:
    - Handle straps (using line detection)
    - Opening/rim (top edge detection)
    - Side panels (for lifting)
    """
    grasps = []

    # Extract ROI
    x, y, w, h = cv2.boundingRect(obj.contour)
    roi = image[y:y+h, x:x+w]

    # Detect vertical/horizontal lines (handles, straps)
    edges = cv2.Canny(roi, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50,
                            minLineLength=30, maxLineGap=10)

    if lines is not None:
        for line in lines:
            # Create grasp at line midpoint
            # ...

    return grasps
```

## Troubleshooting

**No objects detected:**
- Adjust `min_object_area` and `max_object_area`
- Check lighting conditions (adaptive threshold helps but not perfect)
- Verify depth data is valid (`ros2 topic echo /camera/depth/image_raw`)

**Grasp poses are inaccurate:**
- Calibrate camera (check `/camera/color/camera_info`)
- Ensure depth is aligned to RGB
- Increase depth quality settings

**Performance issues:**
- Reduce image resolution
- Decrease contour approximation accuracy
- Disable visualization (`visualize:=false`)

**Invalid depth values:**
- Adjust `depth_min` and `depth_max` range
- Check for reflective/transparent surfaces
- Ensure objects are within camera range

## Example Output

```
[INFO] [opencv_grasp_detector]: OpenCV Grasp Detector initialized
[INFO] [opencv_grasp_detector]: Looking for objects between 1000 and 100000 pixels
[INFO] [opencv_grasp_detector]: Depth range: 0.3m to 2.0m
[INFO] [opencv_grasp_detector]: Detected 2 objects
[INFO] [opencv_grasp_detector]: Best grasp: type=center, pos=(0.123, -0.045, 0.678), confidence=0.80
```

## Next Steps

1. **Test with real objects** - Place objects in front of camera and verify detection
2. **Tune parameters** - Adjust area thresholds for your specific objects
3. **Add ML models** - Integrate YOLO/Detectron2 for semantic understanding
4. **Connect to MoveIt** - Use grasp poses with MTC pick-and-place
5. **Add gripper constraints** - Filter grasps based on gripper width/geometry

## References

- ROS 2 cv_bridge: [http://wiki.ros.org/cv_bridge](http://wiki.ros.org/cv_bridge)
- OpenCV Contours: [https://docs.opencv.org/4.x/d3/d05/tutorial_py_table_of_contents_contours.html](https://docs.opencv.org/4.x/d3/d05/tutorial_py_table_of_contents_contours.html)
- MoveIt Task Constructor: See [MTC_INTEGRATION.md](../../../MTC_INTEGRATION.md)
