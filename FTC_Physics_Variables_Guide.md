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

---

## Additional Tips from FTC Teams (Discord)

### Limelight + Pedro Pathing Integration

**Source:** FTC Discord chat with Team TSFD

**Key Advice:**

1. **Get field position from Limelight** → Convert to Pedro Pathing coordinates → Apply offset from center
   - Convert Limelight's field position to Pedro coordinate system
   - Apply offset from field center for accurate positioning

2. **Refresh Rate Strategy:**
   - Don't update every frame - only at start of match
   - Then again only after getting "slammed" or significant collision
   - Updates are expensive computationally

3. **Relocalization Strategy:**
   - Compensate for distance error from everything
   - OR use "relocalization mode"
   - **Recommended:** Relocalize only when turret is at center position
   - Slam into corner and click button to reset

4. **Turret Setup:**
   - Keep turret at center position for relocalization
   - Makes alignment consistent and easy

**Quote:** "you just get field position from ll and convert it to pedro type cords and then offset from center" - dot (FTC Team TSFD)

---

## VARIABLES TO MEASURE/TUNE - QUICK REFERENCE

### Hardware Measurements (Measure with Tools)

| Variable | Symbol | Typical Value | How to Measure |
|----------|--------|---------------|----------------|
| Flywheel radius | r | 0.04-0.08m | Calipers on wheel |
| Flywheel mass | m | 0.1-0.5kg | Kitchen scale |
| Intake roller radius | r_roller | 0.02-0.05m | Calipers |
| Wheel diameter | d_wheel | 0.08-0.15m | Tape measure |
| Robot launcher height | h_robot | 0.2-0.3m | Ruler from ground |
| Goal height | h_goal | Check game manual | Game manual |
| Field length | - | 3.6m (144") | Game manual |
| Turret gear ratio | G | 3:1 to 10:1 | Count teeth |
| Encoder ticks/rev | - | 2048 (Falcon) | Motor spec |

### Motor Specifications (From Specs)

| Variable | Typical Value | Where to Find |
|----------|---------------|---------------|
| Falcon 500 max RPM | 6000 | REV Robotics spec |
| Falcon 500 ticks/rev | 2048 | REV Robotics spec |
| Kraken max RPM | 6000 | REV Robotics spec |
| GoBILDA电机 RPM | Check spec | goBILDA website |

### Tuning Values (Find Through Testing)

| Variable | Starting Value | How to Tune |
|----------|---------------|-------------|
| K_P (flywheel) | 0.001-0.01 | Increase until oscillation, back off 20% |
| K_I (flywheel) | 0.0-0.001 | Add if steady-state error exists |
| K_D (flywheel) | 0.0-0.001 | Add if overshooting |
| K_F (flywheel) | 12V / max_rpm | From characterization |
| K_S (flywheel) | 0.01-0.1 | Trial and error |
| K_P (turret) | 0.05 | Increase until smooth |
| K_D (turret) | 0.5 | Increase to reduce overshoot |
| Intake speed ratio | S | Test until intaking reliably |

### Calculated Values (From Formula)

These you DON'T need to measure - calculate from other values:

| Variable | Formula |
|----------|---------|
| v_min | √(g × (Δy + √(Δy² + Δx²))) |
| ω (angular velocity) | (RPM × 2π) / 60 |
| v (linear velocity) | ω × r |
| I (moment of inertia) | ½ × m × r² |
| θ (turret angle) | atan2(y_goal - y_robot, x_goal - x_robot) |
| distance | √((x₂-x₁)² + (y₂-y₁)²) |
| degrees_per_tick | (1/ticks) × gear_ratio × 360 |

### Odometry Values (Measure + Calculate)

| Variable | How to Find |
|----------|-------------|
| Wheel circumference | π × diameter (measure) |
| Ticks per inch | ticks_per_rev / (circumference / 25.4) |
| X position | Integrate velocity × cos(heading) × dt |
| Y position | Integrate velocity × sin(heading) × dt |
| Heading | From IMU/gyro |

### What You NEED To Measure:

1. **Flywheel wheel diameter** → Calculate radius
2. **Flywheel weight** → For inertia calculation
3. **Intake roller diameter** → Calculate radius  
4. **Drive wheel diameter** → For odometry
5. **Robot launcher height** → Measure from ground
6. **Goal height** → Check game manual
7. **Turret gear ratio** → Count teeth or measure
8. **Motor encoder ticks** → From motor spec

### What You NEED To Tune (Trial & Error):

1. **All PID gains** - K_P, K_I, K_D, K_F
2. **Intake speed ratio** - S value
3. **Shooting RPM** - For different distances
4. **Hood angles** - For different distances

### What You DON'T Need To Measure (Calculate):

- All trajectory equations
- Angular/linear velocity conversions
- Turret angle from coordinates
- Distance between points
