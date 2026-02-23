# FTC Physics Concepts: Variable Guide
## What to Measure & How to Find It

*This guide explains every variable in the physics equations - what it means, how to measure it, and typical values*

---

## 1. PROJECTILE MOTION VARIABLES

### 1.1 Minimum Velocity (v_min)
```
v_min = √(g × (Δy + √(Δy² + Δx²)))
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **v_min** | Minimum velocity needed to reach goal (m/s) | Calculate using formula |
| **g** | Gravity = 9.81 m/s² | Constant - always 9.81 |
| **Δy** | Vertical distance from robot to goal (m) | Measure: goal height - robot height |
| **Δx** | Horizontal distance from robot to goal (m) | Measure on field or calculate from odometry |

**How to Find Δy:**
- Measure your robot's launcher height from ground (typical: 0.2-0.3m)
- Find goal height in game manual (typical: 0.984m for high goal)
- Δy = goal height - robot height

**How to Find Δx:**
- Use odometry to get robot (x, y) position
- Know goal position on field (from game manual)
- Δx = |robot_x - goal_x|

---

### 1.2 Trajectory Equation
```
y = x × tan(θ) - (g × x²) / (2 × v₀² × cos²(θ))
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **x** | Horizontal position from launch point (m) | Measure or calculate |
| **y** | Vertical position at x (m) | Calculate |
| **θ** | Launch angle (radians) | Measure from mechanism or use desired angle |
| **v₀** | Initial velocity (m/s) | Measure experimentally or calculate from RPM |

**How to Find v₀:**
```
v₀ = ω × r
```
- ω = angular velocity = (RPM × 2π) / 60
- r = flywheel radius (measure your wheel)

---

## 2. FLYWHEEL VARIABLES

### 2.1 Angular Velocity
```
ω = (2π × RPM) / 60
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **ω** | Angular velocity (rad/s) | Calculate from RPM |
| **RPM** | Revolutions per minute | Read from motor encoder |
| **2π** | Constant ≈ 6.283 | Always the same |
| **60** | Seconds per minute | Constant |

### 2.2 Linear Velocity
```
v = ω × r
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **v** | Linear velocity at wheel surface (m/s) | Calculate |
| **ω** | Angular velocity (rad/s) | From RPM formula above |
| **r** | Flywheel radius (m) | **Measure**: radius of your flywheel wheel |

**How to Measure r:**
- Measure wheel diameter with ruler
- r = diameter ÷ 2
- Typical: 0.05-0.1m (2-4 inches)

### 2.3 Moment of Inertia
```
I = (1/2) × m × r²
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **I** | Moment of inertia (kg·m²) | Calculate |
| **m** | Mass of flywheel (kg) | **Weigh** your flywheel |
| **r** | Radius (m) | Measure (see above) |
| **1/2** | Constant for solid disk | Always 0.5 |

**How to Measure m:**
- Use kitchen scale or postal scale
- Typical: 0.1-0.5 kg for FTC flywheel

---

## 3. INTAKE VARIABLES

### 3.1 Roller RPM
```
RPM = (60 × ω) / (2π)
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **RPM** | Roller speed | Calculate or test |
| **ω** | Angular velocity (rad/s) | From motor specs or test |
| **60** | Seconds per minute | Constant |
| **2π** | Constant | Always 6.283 |

### 3.2 Intake Speed Ratio
```
ω_intake = (S × v_drive) / r_roller
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **S** | Speed ratio (roller speed / drivetrain speed) | **Tune through testing** |
| **v_drive** | Drivetrain linear velocity (m/s) | Measure or calculate from motor RPM |
| **r_roller** | Roller radius (m) | **Measure** your intake roller |
| **ω_intake** | Required roller angular velocity | Calculate |

**How to Find v_drive:**
```
v_drive = (motor_RPM × wheel_circumference) / 60
```
- Measure wheel diameter
- Circumference = π × diameter

**How to Find r_roller:**
- Measure roller diameter with calipers
- Divide by 2 for radius

---

## 4. TURRET VARIABLES

### 4.1 Angle to Target
```
θ = atan2(y_target - y_robot, x_target - x_robot)
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **θ** | Angle to target (radians or degrees) | Calculate |
| **y_target** | Goal y-position | From game manual / field layout |
| **y_robot** | Robot y-position | Read from odometry |
| **x_target** | Goal x-position | From game manual |
| **x_robot** | Robot x-position | Read from odometry |
| **atan2** | Arc tangent function | Use math library |

### 4.2 Degrees Per Tick
```
degrees_per_tick = (1 / ticks_per_rev) × gear_ratio × 360
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **ticks_per_rev** | Encoder ticks per revolution | From motor spec (Falcon: 2048) |
| **gear_ratio** | Gear reduction (motor:output) | **Calculate**: measure or from spec |
| **360** | Degrees per revolution | Constant |
| **degrees_per_tick** | What you need | Calculate |

**How to Find Gear Ratio:**
- Count teeth on motor gear ÷ teeth on output gear
- Or: motor_rotations ÷ output_rotations

**Example:**
- Motor has 20 teeth, output has 86 teeth
- Ratio = 86/20 = 4.3:1

---

## 5. PID VARIABLES

### 5.1 PID Output
```
u(t) = K_P × e(t) + K_I × ∫e(τ)dτ + K_D × de(t)/dt + K_F × r(t)
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **K_P** | Proportional gain | **Tune through testing** |
| **K_I** | Integral gain | **Tune through testing** |
| **K_D** | Derivative gain | **Tune through testing** |
| **K_F** | Feedforward gain | From motor characterization |
| **K_S** | Static friction | **Tune through testing** |
| **K_V** | Velocity feedforward | From motor characterization |
| **K_A** | Acceleration feedforward | From motor characterization |
| **e(t)** | Error = setpoint - measurement | Calculate each loop |
| **r(t)** | Setpoint/target | Set in code |

### 5.2 How to Tune PID

**K_P (Proportional):**
1. Set K_I, K_D, K_F to 0
2. Set K_P to 0.01
3. Run system, observe response
4. Increase K_P by 0.01 until oscillation
5. Back off 20%

**K_I (Integral):**
1. Add only if steady-state error exists
2. Start with 0.001
3. Increase until error消除

**K_D (Derivative):**
1. Add if overshooting
2. Start with 0.1
3. Increase until overshoot stops

**K_F (Feedforward):**
- From motor characterization
- Or: K_F = 12V / max_RPM

---

## 6. ODOMETRY VARIABLES

### 6.1 Distance from Encoder
```
distance = (ticks / ticks_per_rev) × wheel_circumference
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **ticks** | Encoder ticks read | Read from motor |
| **ticks_per_rev** | Encoder resolution | From motor spec |
| **wheel_circumference** | π × wheel diameter | **Measure** wheel |
| **distance** | Linear distance traveled | Calculate |

**How to Measure Wheel Circumference:**
- Measure wheel diameter
- Multiply by π (3.14159)
- Typical: 0.3-0.4m (12-15 inches)

### 6.2 Position Update
```
x += (v_left + v_right) / 2 × cos(θ) × dt
y += (v_left + v_right) / 2 × sin(θ) × dt
```

| Variable | What It Is | How to Find |
|----------|-----------|-------------|
| **v_left** | Left wheel velocity | From encoder |
| **v_right** | Right wheel velocity | From encoder |
| **θ** | Robot heading | From IMU/gyro |
| **dt** | Time step (seconds) | System loop time (0.01s for 100Hz) |

---

## 7. MEASUREMENT TOOLS

### What You Need:
1. **Ruler/Measuring Tape** - Field dimensions, robot dimensions
2. **Calipers** - Precise measurements (wheel diameter, roller radius)
3. **Scale** - Weigh flywheel, mechanism
4. **Tuner X** - Motor characterization, RPM reading
5. **Stopwatch** - Timing tests
6. **Protractor** - Angle measurements
7. **Dashboard/Telemetry** - Real-time values

### Typical FTC Values:

| Measurement | Typical Value | Range |
|-------------|---------------|-------|
| Robot height (launcher) | 0.24m | 0.2-0.3m |
| Goal height | 0.984m | Check game manual |
| Flywheel radius | 0.05m | 0.04-0.08m |
| Flywheel mass | 0.2kg | 0.1-0.5kg |
| Wheel diameter | 0.1m (4") | 0.08-0.15m |
| Encoder ticks (Falcon) | 2048 | Check motor spec |
| Max RPM (Falcon 500) | 6000 | Check motor spec |
| Loop time | 0.01s (100Hz) | 0.01-0.035s |

---

## 8. TESTING PROCEDURE

### For Flywheel:
1. Set known RPM using Tuner X
2. Measure actual velocity (strobometer or calc from time)
3. Compare to calculated v = ω × r
4. Adjust for slippage/friction

### For Intake:
1. Run at known power
2. Measure roller RPM with encoder
3. Test intaking at different drivetrain speeds
4. Adjust S ratio until consistent

### For Turret:
1. Command known angle
2. Measure actual angle with protractor or encoder
3. Calculate degrees_per_tick from measurement
4. Verify with multiple angles

### For PID:
1. Plot setpoint vs actual over time
2. Measure settling time, overshoot, steady-state error
3. Adjust gains per tuning procedure
4. Repeat until desired performance

---

## Sources

- REV Robotics - Motor specifications
- GoBILDA - Odometry wheel specifications  
- Game Manual - Field dimensions, goal heights
- WPILib - PID tuning procedures
- FTC Forum - Team testing procedures
