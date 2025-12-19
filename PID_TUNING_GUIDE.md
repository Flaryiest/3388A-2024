# LemLib PID Tuning Guide

This guide will help you tune your robot's PID controllers for optimal autonomous performance.

## Prerequisites
- Robot fully assembled and wired
- LemLib configured correctly (tracking wheels, IMU, motors)
- Testing Auton selected on the brain

## Understanding PID

**PID** stands for Proportional-Integral-Derivative:
- **kP (Proportional)**: Main driving force. Higher = faster reaction to error
- **kD (Derivative)**: Dampening. Reduces oscillation/overshoot
- **kI (Integral)**: Corrects steady-state error (use as last resort)

## Tuning Order

**ALWAYS tune in this order:**
1. ✅ Angular PID (turning)
2. ✅ Lateral PID (driving straight)
3. ✅ Exit Conditions (when to stop motion)

---

## Step 1: Tune Angular PID (Turning)

### Current Settings (in main.cpp):
```cpp
lemlib::ControllerSettings angular_controller(
    2,   // kP - proportional gain
    0,   // kI - integral gain (disable for now)
    10,  // kD - derivative gain
    0,   // anti-windup (disable for now)
    0,   // small error range (disable for now)
    0,   // small error range timeout (disable for now)
    0,   // large error range (disable for now)
    0,   // large error range timeout (disable for now)
    0    // slew (disable for now)
);
```

### Tuning Process:

1. **Run the Testing Auton** - It will do 4 turns: 90°, 180°, 270°, 0°

2. **Watch the robot and follow this flowchart:**

   ```
   Does robot oscillate (wiggle back and forth)?
   ├─ YES → INCREASE kD by 5-10
   └─ NO  → Continue below
   
   Does robot reach target?
   ├─ NO (too slow) → INCREASE kP by 0.5-1
   └─ YES → Continue below
   
   Does robot overshoot target?
   ├─ YES → DECREASE kP by 0.5
   └─ NO  → You're done with kP and kD!
   ```

3. **Typical Values:**
   - kP: 2-6 (higher for heavier robots)
   - kD: 10-30 (increase until oscillation stops)
   - kI: 0 (only add if there's steady-state error)

4. **If robot stops but not at exact target (steady-state error):**
   - Add small kI: 0.01-0.05
   - Set anti-windup to 3 (degrees)
   - This is RARE and usually means kP/kD aren't tuned well

### What to Look For:
- ✅ **Good**: Robot turns smoothly, slows down near target, settles quickly
- ❌ **Bad**: Robot wiggles (oscillates), overshoots, or is very slow

---

## Step 2: Tune Lateral PID (Driving Straight)

**ONLY START THIS AFTER ANGULAR PID IS TUNED!**

### Current Settings (in main.cpp):
```cpp
lemlib::ControllerSettings lateral_controller(
    10,  // kP - proportional gain
    0,   // kI - integral gain (disable for now)
    3,   // kD - derivative gain
    0,   // anti-windup (disable for now)
    0,   // small error range (disable for now)
    0,   // small error range timeout (disable for now)
    0,   // large error range (disable for now)
    0,   // large error range timeout (disable for now)
    0    // slew - acceleration limit (disable for now)
);
```

### Tuning Process:

1. **Uncomment the lateral tests in testing_auton():**
   - Look for the section marked "LATERAL PID TUNING"
   - Remove the `/*` and `*/` around those tests

2. **Run Testing Auton** - It will drive 24", then 48", then back

3. **Watch the robot and follow this flowchart:**

   ```
   Does robot oscillate (jerky movement)?
   ├─ YES → INCREASE kD by 2-5
   └─ NO  → Continue below
   
   Is robot too slow to reach target?
   ├─ YES → INCREASE kP by 2-5
   └─ NO  → Continue below
   
   Does robot overshoot?
   ├─ YES → DECREASE kP by 1-2
   └─ NO  → You're done with kP and kD!
   ```

4. **Check for wheel slippage:**
   - Does the robot position tracking seem off?
   - Do wheels spin/slip during acceleration?
   - If YES: Set slew to 20, then decrease by 2-5 until slippage stops

5. **Typical Values:**
   - kP: 8-15
   - kD: 3-10
   - slew: 0 (if vertical tracking wheel) or 10-20 (if using IMEs)
   - kI: 0 (only if steady-state error)

### What to Look For:
- ✅ **Good**: Robot drives smoothly, slows near target, wheels don't slip
- ❌ **Bad**: Robot jerky, overshoots, wheels slip, or position tracking is off

---

## Step 3: Add Exit Conditions

Exit conditions tell the robot when to stop and move to the next motion.

### Recommended Settings:

```cpp
lemlib::ControllerSettings angular_controller(
    2.8,   // kP (your tuned value)
    0.03,  // kI (if needed)
    25,    // kD (your tuned value)
    3,     // anti-windup (if using kI)
    1,     // small error range (degrees)
    100,   // small error range timeout (ms)
    3,     // large error range (degrees)
    500,   // large error range timeout (ms)
    0      // slew
);

lemlib::ControllerSettings lateral_controller(
    10,    // kP (your tuned value)
    0,     // kI (if needed)
    3,     // kD (your tuned value)
    3,     // anti-windup (if using kI)
    1,     // small error range (inches)
    100,   // small error range timeout (ms)
    3,     // large error range (inches)
    500,   // large error range timeout (ms)
    20     // slew (if needed)
);
```

### What These Mean:
- **Small error range (1 inch/degree)**: If robot is within 1 unit of target for 100ms, exit
- **Large error range (3 inch/degree)**: If robot is within 3 units of target for 500ms, exit
- **Anti-windup (3)**: Only accumulate integral error when within 3 units of target

---

## Step 4: Final Testing

1. **Uncomment the "COMBINED TEST"** in testing_auton()
   - This drives a square pattern
   
2. **Run it and observe:**
   - Does robot complete the square accurately?
   - Does it return to starting position?
   - Are turns and drives smooth?

3. **If position drifts:**
   - Check tracking wheel alignment
   - Ensure tracking wheel isn't slipping
   - Verify tracking wheel diameter is correct in code

---

## Troubleshooting

### Robot oscillates constantly
- **Increase kD** by 5-10 at a time

### Robot is slow/sluggish
- **Increase kP** by 0.5-2

### Robot overshoots target
- **Decrease kP** by 0.5-1

### Wheels slip during acceleration
- **Add slew** (start at 20, decrease until fixed)
- Check that wheels aren't worn out

### Position tracking is inaccurate
- Verify tracking wheel diameter in code
- Check tracking wheel isn't slipping on shaft
- Ensure tracking wheel is perpendicular to ground
- Make sure rotation sensor is wired correctly (port 4)

### Robot doesn't reach exact target (steady-state error)
- Last resort: Add small **kI** (0.01-0.05)
- Set anti-windup to 3
- Usually means kP/kD need better tuning

### Robot crashes into field elements
- Add shorter timeout values in motion commands
- Use exit conditions properly

---

## Quick Reference

### Angular PID Typical Values:
- **kP**: 2-6
- **kD**: 10-30
- **kI**: 0 (or 0.01-0.05 if needed)

### Lateral PID Typical Values:
- **kP**: 8-15
- **kD**: 3-10
- **kI**: 0 (rarely needed)
- **slew**: 0 (vertical tracking) or 10-20 (IMEs)

### Exit Conditions:
- Small range: 1 inch/degree, 100ms
- Large range: 3 inch/degree, 500ms

---

## Advanced: Odometry Calibration

If your position tracking is still off after tuning PID, you may need to calibrate tracking wheel diameter.

### Method 1: Physical Measurement
1. Measure actual wheel diameter with calipers
2. Update in code: `lemlib::Omniwheel::NEW_2` (2" diameter)

### Method 2: Drive Test
1. Mark starting position on field
2. Drive robot 120" (10 tiles) in autonomous
3. Measure actual distance traveled
4. Calculate: `actual_diameter = current_diameter × (120 / measured_distance)`
5. Update wheel diameter in code

---

## How to Use the Testing Auton

1. **Select "Testing Auton"** on the brain selector
2. **Place robot** in open space (48" × 48" minimum)
3. **Start autonomous**
4. **Watch the robot** - observe turns and drives
5. **Adjust values** in main.cpp based on observations
6. **Recompile and upload**
7. **Repeat** until satisfied

Remember: Tuning is iterative! Don't expect perfect results on first try.
