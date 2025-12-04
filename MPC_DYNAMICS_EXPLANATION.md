# MPC Dynamics Model and Constraints

## Overview

The MPC controller uses a **Kinematic Bicycle Model** to predict vehicle motion. This is a simplified model that captures the essential dynamics for path following without the complexity of full vehicle dynamics.

---

## 1. State Variables

The vehicle state is represented by 4 variables:

```
State: [x, y, yaw, speed]
  - x:     Position in x-direction (meters) - forward/backward
  - y:     Position in y-direction (meters) - left/right  
  - yaw:   Vehicle heading angle (radians)
  - speed: Vehicle speed (m/s)
```

**Initial State:**
- Vehicle starts at origin: `[0, 0, 0, current_speed]`
- All predictions are in the **vehicle's local coordinate frame**

---

## 2. Control Inputs

The controller outputs 2 control inputs:

```
Control: [steer, throttle]
  - steer:   Steering angle command [-1, 1]
             -1 = full left, 0 = straight, +1 = full right
  - throttle: Throttle command [0, 0.75]
              0 = no throttle, 0.75 = max throttle
```

---

## 3. Kinematic Bicycle Model

### Model Parameters (from CARLA EgoModel)

```python
front_wb = -0.090769015    # Front wheelbase (meters)
rear_wb = 1.4178275        # Rear wheelbase (meters)
steer_gain = 0.36848336     # Steering gain factor
brake_accel = -4.952399     # Braking acceleration (m/s²)
throt_accel = 0.5633837     # Throttle acceleration (m/s²)
```

### Dynamics Equations

The model uses the following equations to predict future states:

#### Step 1: Compute Wheel Angle
```python
wheel = steer_gain * steer_k
```
Converts normalized steering command to actual wheel angle.

#### Step 2: Compute Slip Angle (β)
```python
beta = atan(rear_wb / (front_wb + rear_wb) * tan(wheel))
```
The **slip angle** accounts for the difference between the vehicle's heading and its velocity direction due to steering geometry.

#### Step 3: Compute Acceleration
```python
if brake:
    accel = brake_accel      # Fixed braking deceleration
else:
    accel = throt_accel * throttle_k  # Proportional to throttle
```

#### Step 4: State Derivatives
```python
x_dot = speed * cos(yaw + beta)      # Velocity in x-direction
y_dot = speed * sin(yaw + beta)      # Velocity in y-direction
yaw_dot = speed / rear_wb * sin(beta) # Angular velocity
speed_dot = accel                     # Acceleration
```

**Key Insight:** The vehicle moves in the direction `(yaw + beta)`, not just `yaw`, because of the slip angle.

#### Step 5: Euler Integration
```python
x_next = x + x_dot * dt
y_next = y + y_dot * dt
yaw_next = yaw + yaw_dot * dt
speed_next = max(0.0, speed + speed_dot * dt)  # Non-negative
```

Where `dt = 0.05` seconds (20 FPS).

---

## 4. Constraints

### Control Constraints

1. **Steering Constraint:**
   ```python
   -1.0 ≤ steer ≤ 1.0
   ```
   Limits steering to physical maximum.

2. **Throttle Constraint:**
   ```python
   0.0 ≤ throttle ≤ 0.75
   ```
   Throttle is non-negative and capped at 0.75 (from config).

### State Constraints

3. **Speed Constraint:**
   ```python
   mpc_min_speed (0.0) ≤ speed ≤ mpc_max_speed (13.41 m/s = 30 mph)
   ```
   Enforces speed limits.

4. **Non-Negative Speed:**
   ```python
   speed ≥ 0.0
   ```
   Vehicle cannot move backward (enforced via `ca.fmax(0.0, ...)`).

### Dynamics Constraints

5. **State Transition:**
   ```python
   X[k+1] = f(X[k], U[k])
   ```
   Each state must follow the kinematic bicycle model equations.

6. **Initial State:**
   ```python
   X[0] = [0, 0, 0, current_speed]
   ```
   Must start at current vehicle state.

---

## 5. Cost Function

The MPC minimizes a weighted sum of costs:

### Tracking Error Cost
```python
tracking_error = (x - wp_x)² + (y - wp_y)²
cost += mpc_q_tracking (10.0) * tracking_error
```
Penalizes deviation from predicted waypoints.

### Speed Tracking Cost
```python
speed_error = (speed - desired_speed)²
cost += mpc_q_speed (5.0) * speed_error
```
Penalizes deviation from desired speed.

### Control Effort Cost
```python
cost += mpc_r_steer (1.0) * steer²
cost += mpc_r_throttle (0.5) * throttle²
```
Penalizes large control inputs (smoothness).

### Terminal Cost
```python
terminal_error = (x_final - wp_final_x)² + (y_final - wp_final_y)²
cost += mpc_q_terminal (20.0) * terminal_error
```
Extra weight on reaching the final waypoint.

---

## 6. Optimization Problem

**Formulation:**
```
minimize:  Σ [tracking_cost + speed_cost + control_cost] + terminal_cost
subject to:
  - Dynamics: X[k+1] = f(X[k], U[k])
  - Initial: X[0] = [0, 0, 0, current_speed]
  - Steering: -1 ≤ steer ≤ 1
  - Throttle: 0 ≤ throttle ≤ 0.75
  - Speed: 0 ≤ speed ≤ 30 mph
```

**Horizon:** N = 4 steps (0.2 seconds ahead at 20 FPS)

**Solver:** IPOPT (via CasADi) or SLSQP (via scipy)

---

## 7. Key Design Choices

### Why Kinematic Bicycle Model?

1. **Simplicity:** No need for tire forces, friction, or complex dynamics
2. **Speed:** Fast enough for real-time control (20 FPS)
3. **Accuracy:** Sufficient for path following at moderate speeds
4. **Matching:** Uses same model as CARLA's EgoModel for consistency

### Why Slip Angle?

The slip angle `β` accounts for the fact that when you steer, the vehicle doesn't immediately turn in the steering direction. The rear wheels follow a different path than the front wheels, creating a "slip" between heading and velocity direction.

### Why 4-Step Horizon?

- **Short enough:** Fast computation (< 50ms per step)
- **Long enough:** Captures turning dynamics (0.2s ahead)
- **Tunable:** Can increase if needed (config parameter)

---

## 8. Code Location

- **Dynamics Model:** `model.py` lines 834-863
- **Constraints:** `model.py` lines 884-887
- **Cost Function:** `model.py` lines 865-882
- **Parameters:** `config.py` lines 206-216

---

## 9. Visualization

```
Time Step k:
  State: [x_k, y_k, yaw_k, speed_k]
  Control: [steer_k, throttle_k]
         ↓
  Compute: wheel, beta, accel
         ↓
  Derivatives: [x_dot, y_dot, yaw_dot, speed_dot]
         ↓
  Euler Integration
         ↓
Time Step k+1:
  State: [x_{k+1}, y_{k+1}, yaw_{k+1}, speed_{k+1}]
```

The MPC optimizes all N steps simultaneously to find the best control sequence that minimizes cost while satisfying all constraints.

---

## 10. Differences from PID

| Aspect | PID Controller | MPC Controller |
|--------|---------------|----------------|
| **Prediction** | None (reactive) | 4 steps ahead |
| **Model** | None | Kinematic bicycle |
| **Optimization** | Proportional gains | Full optimization |
| **Constraints** | None | Hard constraints |
| **Cost** | Implicit | Explicit multi-objective |
| **Smoothness** | Depends on gains | Enforced via cost |

MPC is more sophisticated but computationally heavier. It can anticipate turns and adjust speed/steering proactively.

