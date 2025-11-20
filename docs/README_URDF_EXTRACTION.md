# URDF Component Extraction Script

## Overview

`extract_urdf_components.py` is a utility script that extracts joints and links from a URDF file into separate xacro files. This is useful for modularizing robot descriptions and maintaining clean, organized URDF/xacro structure.

## Features

- Extracts all `<link>` elements into a separate xacro file
- Extracts all `<joint>` elements into a separate xacro file
- Maintains proper XML formatting and indentation (4 spaces)
- Preserves all element attributes and nested content
- Generates valid xacro files with proper headers
- Automatically fixes mesh paths from `package://humanoid_arm_5dof/meshes/` to `package://arm_description/meshes/visual/`

## Usage

### Basic Usage

```bash
python3 extract_urdf_components.py <input_urdf> <output_joints_xacro> <output_links_xacro>
```

### Example

Extract from the humanoid arm URDF:

```bash
python3 extract_urdf_components.py \
    src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf \
    src/robot_description/arm_description/urdf/joints/arm_joints.xacro \
    src/robot_description/arm_description/urdf/links/arm_links.xacro
```

### Verbose Mode

Use the `-v` or `--verbose` flag to see detailed extraction information:

```bash
python3 extract_urdf_components.py -v \
    src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf \
    src/robot_description/arm_description/urdf/joints/arm_joints.xacro \
    src/robot_description/arm_description/urdf/links/arm_links.xacro
```

Output:
```
Extracting links from src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf...
Found 23 links
Extracting joints from src/robot_description/arm_description/urdf/humanoid_arm_5dof.urdf...
Found 22 joints
Writing links to src/robot_description/arm_description/urdf/links/arm_links.xacro...
Writing joints to src/robot_description/arm_description/urdf/joints/arm_joints.xacro...
✓ Successfully extracted 23 links and 22 joints
  Links:  src/robot_description/arm_description/urdf/links/arm_links.xacro
  Joints: src/robot_description/arm_description/urdf/joints/arm_joints.xacro
```

### Help

```bash
python3 extract_urdf_components.py --help
```

## Output Format

The script generates two xacro files:

### Links File (arm_links.xacro)
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <link name="base_link">
        <inertial>
            <origin xyz="-1.4142E-08 1.4615E-08 0.35593" rpy="0 0 0" />
            <mass value="77.718" />
            ...
        </inertial>
        ...
    </link>
    ...
</robot>
```

### Joints File (arm_joints.xacro)
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <joint name="left_shoulder_pitch_rs04" type="revolute">
        <origin xyz="0 0 0" rpy="0 0 0" />
        <parent link="left_shoulder_pitch_rs04_actuator" />
        <child link="left_shoulder_pitch_rs04_flange" />
        <axis xyz="1 0 0" />
        <limit lower="-3.14" upper="3.14" effort="120" velocity="3" />
        ...
    </joint>
    ...
</robot>
```

## Requirements

- Python 3.x (tested with Python 3.10+)
- Standard library only (no external dependencies)

## Notes

- The script automatically creates output directories if they don't exist
- All XML elements are formatted with 4-space indentation
- The script preserves all attributes, nested elements, and content from the original URDF
- Generated files are valid xacro files that can be included in other xacro files using `<xacro:include>`

## Integration Example

After extracting, you can include these files in your main xacro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm_description">
    <!-- Include extracted components -->
    <xacro:include filename="$(find arm_description)/urdf/links/arm_links.xacro" />
    <xacro:include filename="$(find arm_description)/urdf/joints/arm_joints.xacro" />

    <!-- Include other components -->
    <xacro:include filename="$(find arm_description)/urdf/macros/ros2_control.xacro" />
</robot>
```
