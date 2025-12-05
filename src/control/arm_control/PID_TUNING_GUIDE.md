# PID Tuning Guide for Arm Controller

## Current Status: 🔴 OSCILLATING → Fixed with conservative gains

**Problem:** Robot arm having "seizures" (high-frequency oscillation)
**Cause:** Initial PID gains were too aggressive (Kp=100, Kd=10)
**Solution:** Reduced gains by 10x (Kp=10, Kd=2, Ki=0)

---

## How to Test Changes

### 1. Stop Current Simulation
```bash
# Find and kill the simulation process
pkill -f "full_system"
# Or use Ctrl+C in the terminal
```

### 2. Update Config Files
Config files are symlinked, so just edit them directly:
- `src/control/arm_control/config/controllers.yaml`
- `src/simulation/arm_gazebo/config/controllers.yaml`

No rebuild needed for config changes! ✨

### 3. Restart Simulation
```bash
# Using GUI (recommended)
ros2 run arm_gui_tools full_system_launcher.py

# Or command line
ros2 launch arm_system_bringup full_system.launch.py
```

### 4. Verify Controller is Active
```bash
ros2 control list_controllers
# Should show: arm_controller [arm_control/ArmController] active
```

### 5. Observe Behavior
Open Gazebo GUI to watch the arm:
```bash
gz sim -g
```

**What to look for:**
- ✅ **Stable:** Arm stays still without shaking → Good!
- ⚠️ **Drifting:** Arm slowly moves away → Increase Kp
- 🔴 **Oscillating:** Arm vibrates/shakes → Decrease Kp or increase Kd

---

## PID Tuning Process

### Current Gains (Conservative - should be stable)
```yaml
kp: [10.0, 10.0, 8.0, 8.0, 5.0, 5.0]
ki: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Disabled for now
kd: [2.0, 2.0, 1.5, 1.5, 1.0, 1.0]
```

### Step-by-Step Tuning

#### **Phase 1: Stabilize (Current)**
**Goal:** Make arm stop oscillating

1. ✅ Start with low gains (Kp=10, Ki=0, Kd=2)
2. Check if arm is stable when simulation starts
3. If still oscillating → reduce Kp by 50%
4. If stable → proceed to Phase 2

**Expected behavior:** Arm should stay perfectly still

---

#### **Phase 2: Increase Response (Next)**
**Goal:** Make arm respond faster without oscillating

**For each joint (or all at once):**

1. **Increase Kp gradually:**
   ```yaml
   kp: [15.0, 15.0, 12.0, 12.0, 8.0, 8.0]  # +50%
   ```
   - Restart simulation
   - Check for oscillations
   - If stable, increase more
   - If oscillates, back off

2. **Target Kp range:** 20-50 (depends on robot dynamics)

3. **Signs you've gone too far:**
   - High-frequency vibration
   - "Ringing" after disturbances
   - Audible humming/buzzing

---

#### **Phase 3: Add Damping (If Needed)**
**Goal:** Reduce overshoot and settling time

If increasing Kp causes oscillation:

1. **Increase Kd:**
   ```yaml
   kd: [3.0, 3.0, 2.5, 2.5, 1.5, 1.5]  # +50%
   ```

2. **Kd adds damping** (like shock absorbers)
   - Too low: Overshoot, slow settling
   - Too high: Sluggish response, noise amplification

3. **Good Kd range:** Usually 10-20% of Kp value

---

#### **Phase 4: Eliminate Steady-State Error (Later)**
**Goal:** Make arm reach exact target position

Once Kp and Kd are tuned:

1. **Add small Ki:**
   ```yaml
   ki: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
   ```

2. **Ki eliminates steady-state error** but can cause:
   - Integral windup
   - Overshoot
   - Slow oscillations

3. **Keep Ki small:** Usually 0.01-0.1

---

## Quick Tuning Rules

### The "Ziegler-Nichols" Shortcut
1. Set Ki=0, Kd=0
2. Increase Kp until arm oscillates continuously
3. Call that Kp_critical, measure oscillation period T
4. Set:
   - Kp = 0.6 × Kp_critical
   - Ki = 1.2 × Kp_critical / T
   - Kd = 0.075 × Kp_critical × T

### The "Feel-Based" Method (Easier)
1. **Too much oscillation** → Reduce Kp by 20-50%
2. **Too slow response** → Increase Kp by 20-50%
3. **Overshoot/ringing** → Increase Kd by 20-50%
4. **Steady-state error** → Add small Ki (start 0.01)

---

## Testing Tools

### Monitor Joint States
```bash
# Watch position, velocity, effort in real-time
ros2 topic echo /joint_states
```

### Monitor Specific Joint
```bash
# Example: Watch shoulder pitch
ros2 topic echo /joint_states --field position[0]
```

### Apply Disturbance (Manual Test)
In Gazebo GUI:
1. Click on the arm link
2. Apply force (drag with mouse)
3. Observe how quickly it returns to position

**Good PID:** Returns quickly without oscillating
**Bad PID:** Oscillates or drifts away

---

## Expected Performance Metrics

### ✅ Good PID Tuning
- **Rise time:** < 1 second to reach target
- **Overshoot:** < 10% of step size
- **Settling time:** < 2 seconds
- **Steady-state error:** < 0.01 rad (< 0.6°)
- **No oscillations** during hold

### ⚠️ Needs Improvement
- Rise time > 2 seconds → Increase Kp
- Overshoot > 20% → Increase Kd or reduce Kp
- Settling time > 3 seconds → Tune Kd
- Visible oscillations → Reduce Kp

---

## Per-Joint Tuning

Different joints may need different gains:

### Shoulder Joints (Larger, More Inertia)
- Need **higher Kp** for responsiveness
- Need **higher Kd** for damping
- Current: Kp=10, Kd=2

### Wrist Joints (Smaller, Less Inertia)
- Need **lower gains** to avoid oscillation
- More sensitive to noise
- Current: Kp=5, Kd=1

**Strategy:** Tune shoulder first, then adjust wrist proportionally

---

## Troubleshooting

### Problem: Arm immediately falls down
**Cause:** Gains too low, can't support gravity
**Fix:** Increase Kp significantly (double it)

### Problem: Arm shakes violently
**Cause:** Gains too high
**Fix:** Reduce Kp by 50%, reduce Kd by 50%

### Problem: Arm drifts slowly
**Cause:** No integral term, or gravity not compensated
**Fix:** Add small Ki (0.01-0.1)

### Problem: Arm oscillates slowly (< 1 Hz)
**Cause:** Ki too high (integral windup)
**Fix:** Reduce Ki by 50% or disable

### Problem: Different joints behave differently
**Cause:** Need per-joint tuning
**Fix:** Tune each joint individually

---

## Current Tuning Log

### Iteration 1 (Initial - TOO HIGH)
```yaml
kp: [100.0, 100.0, 80.0, 80.0, 50.0, 50.0]
ki: [0.1, 0.1, 0.05, 0.05, 0.01, 0.01]
kd: [10.0, 10.0, 8.0, 8.0, 5.0, 5.0]
```
**Result:** 🔴 Severe oscillation ("robot seizure")

### Iteration 2 (Conservative - Current)
```yaml
kp: [10.0, 10.0, 8.0, 8.0, 5.0, 5.0]
ki: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
kd: [2.0, 2.0, 1.5, 1.5, 1.0, 1.0]
```
**Result:** ⏳ Testing now...

### Iteration 3 (To be determined)
```yaml
# Add your tuned values here after testing
kp: [?, ?, ?, ?, ?, ?]
ki: [?, ?, ?, ?, ?, ?]
kd: [?, ?, ?, ?, ?, ?]
```
**Result:** _Document what you observe_

---

## Next Steps After Stable

Once you have stable PID gains:

1. **Add trajectory following** - Make arm actually move to commands
2. **Add joint limits** - Prevent damage from over-rotation
3. **Add effort limits** - Prevent excessive torque
4. **Test with motion planner** - Run example.py to see tracking

---

## Quick Reference Card

```
╔════════════════════════════════════════════╗
║         PID Quick Tuning Guide             ║
╠════════════════════════════════════════════╣
║ Symptom              → Action              ║
╠════════════════════════════════════════════╣
║ Oscillates fast      → Reduce Kp           ║
║ Oscillates slow      → Reduce Ki           ║
║ Slow response        → Increase Kp         ║
║ Overshoots           → Increase Kd         ║
║ Drifts away          → Increase Kp or Ki   ║
║ Noisy/jittery        → Reduce Kd           ║
║ Falls down           → Increase Kp (2x)    ║
╚════════════════════════════════════════════╝
```

**Remember:** Change one parameter at a time, test, document results!

---

**Last Updated:** 2025-12-04
**Status:** Conservative gains applied, awaiting test results
