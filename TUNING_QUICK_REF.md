# PID Tuning Quick Reference Card

## 📋 Tuning Checklist

- [ ] Angular PID tuned (turning)
- [ ] Lateral PID tuned (driving)
- [ ] Exit conditions configured
- [ ] Combined test successful
- [ ] Odometry verified accurate

---

## 🎯 Angular PID (Turning)

**Test:** `chassis.turnToHeading(90, 100000);`

| Problem | Solution |
|---------|----------|
| 🔄 Oscillates (wiggles) | ↑ INCREASE kD (+5-10) |
| 🐢 Too slow | ↑ INCREASE kP (+0.5-1) |
| 🚀 Overshoots | ↓ DECREASE kP (-0.5) |
| 📍 Stops but not at target | ADD kI (0.01-0.05) + anti-windup=3 |

**Typical values:**
```cpp
kP: 2-6
kD: 10-30
kI: 0 (or 0.01-0.05)
```

---

## 🎯 Lateral PID (Driving Straight)

**Test:** `chassis.moveToPoint(0, 48, 100000);`

| Problem | Solution |
|---------|----------|
| 🔄 Jerky/oscillates | ↑ INCREASE kD (+2-5) |
| 🐢 Too slow | ↑ INCREASE kP (+2-5) |
| 🚀 Overshoots | ↓ DECREASE kP (-1-2) |
| 💨 Wheels slip | ADD slew=20, then ↓ DECREASE (-2-5) |
| 📍 Stops but not at target | ADD kI (0.01-0.05) + anti-windup=3 |

**Typical values:**
```cpp
kP: 8-15
kD: 3-10
kI: 0 (rarely needed)
slew: 0 (w/ tracking) or 10-20 (w/ IMEs)
```

---

## 🎮 Testing Process

### 1️⃣ Angular Tuning
```
1. Select "Testing Auton" on brain
2. Watch 4 turns (90°, 180°, 270°, 0°)
3. Adjust angular_controller values
4. Recompile & repeat until smooth
```

### 2️⃣ Lateral Tuning
```
1. Uncomment lateral tests in testing_auton()
2. Watch drives (24", 48", back to 0)
3. Adjust lateral_controller values
4. Recompile & repeat until smooth
```

### 3️⃣ Combined Test
```
1. Uncomment square test
2. Verify robot returns to start
3. If not, check odometry calibration
```

---

## 🔧 Exit Conditions

After PID is tuned, add these:

```cpp
// Angular
small error: 1 degree, 100ms
large error: 3 degrees, 500ms

// Lateral  
small error: 1 inch, 100ms
large error: 3 inches, 500ms
```

---

## ⚠️ Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Can't stop oscillating | kD too low | Keep increasing kD |
| Position drift over time | Tracking wheel slip | Check wheel, verify diameter |
| Inconsistent performance | Battery voltage | Use fresh battery for tuning |
| Robot tips forward | Slew too high | Decrease slew |
| Turns are good, drives bad | Lateral PID needs tuning | Tune lateral separately |

---

## 📊 Where to Edit Values

**File:** `src/main.cpp`

**Lines ~66-90:** Look for these sections:
```cpp
lemlib::ControllerSettings lateral_controller(
    10,  // ← Edit kP here
    0,   // ← Edit kI here
    3,   // ← Edit kD here
    ...
);

lemlib::ControllerSettings angular_controller(
    2,   // ← Edit kP here
    0,   // ← Edit kI here
    10,  // ← Edit kD here
    ...
);
```

---

## 💡 Pro Tips

1. **Always use fresh battery** when tuning
2. **Tune on flat surface** (field tiles ideal)
3. **Make small adjustments** (don't jump from 10 to 50)
4. **Test multiple distances** (near, medium, far)
5. **Save known-good values** before experimenting
6. **Angular first, lateral second** - order matters!

---

## 🎓 Understanding the Values

**kP (Proportional)**
- How strongly robot reacts to being off target
- Higher = faster but might overshoot
- Lower = slower but more stable

**kD (Derivative)**  
- Dampens movement, prevents oscillation
- Higher = less wiggling
- Too high = sluggish response

**kI (Integral)**
- Corrects accumulated error
- Use ONLY if robot consistently stops short
- Can cause overshoot if too high

**Slew**
- Limits acceleration (0 = unlimited)
- Prevents wheel slip and tipping
- Higher = faster acceleration allowed

---

## 📞 Still Having Issues?

1. Check LemLib documentation: https://lemlib.readthedocs.io/
2. Verify hardware:
   - Tracking wheel not slipping
   - IMU is calibrating (light stops blinking)
   - Motors are wired correctly
   - Rotation sensor in port 4
3. Check initialization:
   - `chassis.calibrate()` is called
   - Pose is set before movements: `chassis.setPose(0, 0, 0)`

---

**Remember:** Tuning is iterative. Don't expect perfection immediately!

Good luck! 🚀
