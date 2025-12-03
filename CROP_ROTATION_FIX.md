# Debug Crops Rotation Fix

## The Problem

In `simple_detect.py`, the **Debug: Crops** window shows objects that should be rotated to vertical, but sometimes they remain tilted.

### Root Causes

1. **Threshold too high** (line 168):
   ```python
   if has_orientation and crop.size > 0 and abs(rotation_angle) > 2:
   ```
   Objects with small detected angles (< 2°) won't be rotated at all!

2. **Angle detection inconsistency**: The `minAreaRect` approach can be ambiguous for certain shapes

3. **No visual feedback**: Hard to tell if rotation is working

## The Solution: `simple_detect_improved.py`

### Key Improvements

1. **Always rotates** (removed threshold):
   ```python
   if enable_rotation and has_orientation and crop.size > 0:
       # Always rotate, no threshold check
   ```

2. **Better orientation detection**:
   - Uses both `minAreaRect` AND `image moments`
   - More robust for different object shapes
   - Handles ambiguous cases better

3. **Visual feedback in crops**:
   - Shows "ROTATED ✓" in green when rotation applied
   - Displays orientation angle, tilt, and rotation amount
   - Draws vertical reference line in rotated crops

4. **Toggle feature** - Press 'r' to enable/disable rotation for comparison

### Usage

```bash
# Run the improved version
python3 simple_detect_improved.py

# Controls:
#   'q' - Quit
#   'd' - Toggle debug info
#   'c' - Toggle crop window
#   'r' - Toggle rotation ON/OFF (compare before/after)
```

## Visual Comparison

### Before (simple_detect.py):
```
┌─ Debug: Crops ─────────────┐
│                             │
│   ID:7 tilt:30.6°          │
│   ╱                         │
│  ╱  Object still tilted!    │
│ ╱                           │
│                             │
└─────────────────────────────┘
```

### After (simple_detect_improved.py):
```
┌─ Debug: Crops (Auto-Rotated) ──┐
│     ↑  ID:7 ROTATED ✓          │
│     │  Orient: 120.6°           │
│     │  Tilt: 30.6°              │
│     │  Rotated by: -30.6°       │
│     │                           │
│     │  Object now vertical! ✓   │
│     │                           │
└─────────────────────────────────┘
```

## What Changed in the Code

### Original (simple_detect.py):

```python
# Line 146-157
if has_orientation:
    display_tilt = long_axis_angle - 90.0
    rotation_angle = -display_tilt

    # PROBLEM: Clamps to prevent "crazy values"
    if rotation_angle > 89.0:
        rotation_angle = 89.0
    if rotation_angle < -89.0:
        rotation_angle = -89.0

# Line 168
# PROBLEM: Only rotates if angle > 2°
if has_orientation and crop.size > 0 and abs(rotation_angle) > 2:
    crop_upright = cv2.warpAffine(crop, M, (new_w, new_h))
```

### Improved (simple_detect_improved.py):

```python
# Better orientation calculation
orientation_angle, _ = get_orientation_from_mask(mask, frame.shape)
tilt_from_vertical = orientation_angle - 90.0
rotation_angle = -tilt_from_vertical

# Handle 180° ambiguity
if abs(tilt_from_vertical) > 90:
    tilt_from_vertical = adjust_for_ambiguity(tilt_from_vertical)
    rotation_angle = -tilt_from_vertical

# ALWAYS rotate (no threshold)
if enable_rotation and has_orientation and crop.size > 0:
    crop_upright = cv2.warpAffine(crop, M, (new_w, new_h),
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=(0, 0, 0))
```

## Testing the Fix

### Step 1: Run improved version
```bash
python3 simple_detect_improved.py
```

### Step 2: Test with tilted object
1. Hold an object (bottle, phone, box) at various angles
2. Watch the **Debug: Crops** window
3. Object should **always appear vertical** in the crop

### Step 3: Compare with/without rotation
1. Press 'r' to toggle rotation OFF
2. See object tilted in crop (original behavior)
3. Press 'r' again to toggle rotation ON
4. See object straightened in crop (fixed behavior)

## Understanding the Angles

The improved version shows three key angles:

1. **Orient**: Angle of object's long axis from horizontal (0-180°)
   - 0° = horizontal right
   - 90° = vertical up
   - 180° = horizontal left

2. **Tilt**: How far the object is from vertical (Orient - 90°)
   - 0° = perfectly vertical
   - Positive = tilted left
   - Negative = tilted right

3. **Rotated by**: Angle applied to crop to make it vertical
   - Always equals -Tilt
   - This is the correction angle

### Example:
```
Orient: 120°     (object leans left by 30°)
Tilt: 30°        (120° - 90° = 30°)
Rotated by: -30° (rotate right to correct)
Result: Vertical crop! ✓
```

## Why This Matters for Your Workflow

### Original Problem:
You need the **Debug: Crops** to always show objects upright because:
1. Better for visual inspection
2. Easier to re-run detection on normalized crops
3. Consistent orientation for downstream tasks

### Solution Benefits:
1. **Crops always upright**: No matter how you tilt the object
2. **Visual confirmation**: See rotation status and angles
3. **Debugging toggle**: Compare rotated vs. original with 'r' key
4. **More robust**: Works with more object shapes and angles

## Troubleshooting

### Issue: Crop shows object at 90° angle (sideways)

**Cause**: Orientation ambiguity - algorithm detected wrong axis as "long"

**Solution**: The improved version handles this with ambiguity resolution:
```python
if abs(tilt_from_vertical) > 90:
    # Object is actually rotated 180° - flip it
    tilt_from_vertical = tilt_from_vertical - 180 if tilt_from_vertical > 0 else tilt_from_vertical + 180
```

### Issue: Crop rotation seems jittery/unstable

**Cause**: Mask boundaries changing between frames

**Solution**: Add temporal smoothing (future enhancement):
```python
# Smooth angle over last N frames
smoothed_angle = moving_average(rotation_angles[-5:])
```

### Issue: Rectangular objects rotate wrong

**Cause**: Width vs. height ambiguity in `minAreaRect`

**Solution**: The improved version uses image moments as fallback method

## Next Steps: 3D Orientation

For even better results, consider using the 3D approach:

1. **Current (2D)**: Detects tilt in camera image plane only
2. **Upgraded (3D)**: Detects true 3D orientation from depth data

See `simple_detect_3d.py` and `ORIENTATION_COMPARISON.md` for details!

---

**Quick Start:**
```bash
python3 simple_detect_improved.py
# Press 'c' to show crops
# Press 'r' to toggle rotation
# Tilt objects - watch crops stay vertical!
```
