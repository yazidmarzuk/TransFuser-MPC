# TransFuser Code Walkthrough - Step by Step

This document shows the actual code that executes at each step of the inference pipeline.

---

## **STEP 1: Main Entry Point - `run_step()`**

**File:** `submission_agent.py`  
**Lines:** 236-398

This is called by CARLA every frame (20 FPS).

```python
@torch.inference_mode()
def run_step(self, input_data, timestamp):
    self.step += 1
    
    # Initialize on first call
    if not self.initialized:
        self._init()  # Sets up route planner
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        return control
    
    # STEP 2: Process sensor data
    tick_data = self.tick(input_data)
    
    # Skip every other frame (LiDAR runs at 10Hz, sim at 20Hz)
    if self.step % self.config.action_repeat == 1:
        self.update_gps_buffer(self.control, tick_data['compass'], tick_data['speed'])
        return self.control
    
    # STEP 3: Prepare inputs for neural network
    image = self.prepare_image(tick_data)
    lidar_bev = self.prepare_lidar(tick_data)
    target_point_image, target_point = self.prepare_goal_location(tick_data)
    velocity = gt_velocity.reshape(1, 1)
    
    # STEP 4: Check if stuck
    is_stuck = False
    if(self.stuck_detector > self.config.stuck_threshold and 
       self.forced_move < self.config.creep_duration):
        is_stuck = True
        self.forced_move += 1
    
    # STEP 5: Neural network forward pass
    with torch.no_grad():
        pred_wp, _ = self.nets[0].forward_ego(
            image, lidar_bev, target_point, target_point_image, velocity, ...)
    
    # STEP 6: Post-process waypoints
    self.pred_wp = torch.stack(pred_wps, dim=0).mean(dim=0)  # Ensemble average
    
    # STEP 7: Run MPC controller
    steer, throttle, brake = self.nets[0].control_mpc(
        self.pred_wp, gt_velocity, is_stuck)
    
    # STEP 8: Safety checks and final control
    control = carla.VehicleControl()
    control.steer = float(steer)
    control.throttle = float(throttle)
    control.brake = float(brake)
    
    return control
```

---

## **STEP 2: Sensor Data Processing - `tick()`**

**File:** `submission_agent.py`  
**Lines:** 184-234

Processes raw sensor data from CARLA.

```python
def tick(self, input_data):
    # Extract RGB images from 3 cameras
    rgb = []
    for pos in ['left', 'front', 'right']:
        rgb_cam = 'rgb_' + pos
        rgb_pos = cv2.cvtColor(input_data[rgb_cam][1][:, :, :3], cv2.COLOR_BGR2RGB)
        rgb_pos = self.scale_crop(Image.fromarray(rgb_pos), ...)
        rgb.append(rgb_pos)
    rgb = np.concatenate(rgb, axis=1)  # Concatenate horizontally
    
    # Extract GPS, speed, compass
    gps = input_data['gps'][1][:2]
    speed = input_data['speed'][1]['speed']
    compass = input_data['imu'][1][-1]
    
    # Extract LiDAR
    if (self.backbone != 'latentTF'):
        lidar = input_data['lidar'][1][:, :3]
    
    # Denoise GPS using buffer
    pos = self._get_position(result)
    self.gps_buffer.append(pos)
    denoised_pos = np.average(self.gps_buffer, axis=0)
    
    # Route planning - get next waypoint
    waypoint_route = self._route_planner.run_step(denoised_pos)
    next_wp, next_cmd = waypoint_route[1] if len(waypoint_route) > 1 else waypoint_route[0]
    
    # Convert next waypoint to vehicle-local coordinates
    theta = compass + np.pi/2
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    local_command_point = np.array([next_wp[0]-denoised_pos[0], 
                                    next_wp[1]-denoised_pos[1]])
    local_command_point = R.T.dot(local_command_point)
    result['target_point'] = tuple(local_command_point)
    
    return result
```

---

## **STEP 3: Image Preprocessing - `prepare_image()`**

**File:** `submission_agent.py`  
**Lines:** 485-493

```python
def prepare_image(self, tick_data):
    image = Image.fromarray(tick_data['rgb'])
    image_degrees = []
    for degree in self.aug_degrees:  # Currently [0]
        crop_shift = degree / 60 * self.config.img_width
        rgb = torch.from_numpy(
            self.shift_x_scale_crop(image, 
                                   scale=self.config.scale, 
                                   crop=self.config.img_resolution, 
                                   crop_shift=crop_shift)
        ).unsqueeze(0)
        image_degrees.append(rgb.to('cuda', dtype=torch.float32))
    image = torch.cat(image_degrees, dim=0)
    return image  # Shape: [1, 3, 160, 704]
```

---

## **STEP 4: LiDAR Preprocessing - `prepare_lidar()`**

**File:** `submission_agent.py`  
**Lines:** 545-551

```python
def prepare_lidar(self, tick_data):
    lidar_transformed = deepcopy(tick_data['lidar']) 
    lidar_transformed[:, 1] *= -1  # Invert y-axis (CARLA to model coords)
    # Convert point cloud to BEV histogram
    lidar_transformed = torch.from_numpy(
        lidar_to_histogram_features(lidar_transformed)
    ).unsqueeze(0)
    lidar_transformed_degrees = [lidar_transformed.to('cuda', dtype=torch.float32)]
    lidar_bev = torch.cat(lidar_transformed_degrees[::-1], dim=1)
    return lidar_bev  # Shape: [1, 2, 256, 256]
```

---

## **STEP 5: Target Point Preparation - `prepare_goal_location()`**

**File:** `submission_agent.py`  
**Lines:** 553-575

```python
def prepare_goal_location(self, tick_data):
    tick_data['target_point'] = [
        torch.FloatTensor([tick_data['target_point'][0]]),
        torch.FloatTensor([tick_data['target_point'][1]])
    ]
    target_point = torch.stack(tick_data['target_point'], dim=1).to('cuda')
    
    # Create target point image (draws point on BEV)
    target_point_image = draw_target_point(current_target_point[0])
    target_point_image = torch.from_numpy(target_point_image)[None].to('cuda')
    
    return target_point_image, target_point
```

---

## **STEP 6: Neural Network Forward Pass - `forward_ego()`**

**File:** `model.py`  
**Lines:** 990-1038

```python
def forward_ego(self, rgb, lidar_bev, target_point, target_point_image, ego_vel, ...):
    
    # Optionally add target point to LiDAR BEV
    if self.use_target_point_image:
        lidar_bev = torch.cat((lidar_bev, target_point_image), dim=1)
    
    # PERCEPTION: Multi-scale transformer fusion
    if (self.backbone == 'transFuser'):
        features, image_features_grid, fused_features = self._model(
            rgb, lidar_bev, ego_vel)
    # Returns:
    # - features: BEV features for object detection
    # - image_features_grid: Image features for multitask
    # - fused_features: Combined features for waypoint prediction
    
    # WAYPOINT PREDICTION: GRU autoregressive generation
    pred_wp, _, _, _, _ = self.forward_gru(fused_features, target_point)
    # Returns: [B, 4, 2] - 4 future waypoints in (x, y)
    
    # OBJECT DETECTION: CenterNet head
    preds = self.head([features[0]])
    results = self.head.get_bboxes(preds[0], preds[1], preds[2], ...)
    bboxes, _ = results[0]
    # Returns: Bounding boxes with [x, y, w, h, yaw, speed, brake, confidence]
    
    # Filter by confidence
    bboxes = bboxes[bboxes[:, -1] > self.config.bb_confidence_threshold]
    
    return pred_wp, rotated_bboxes
```

---

## **STEP 7: Waypoint Generation - `forward_gru()`**

**File:** `model.py`  
**Lines:** 628-663

```python
def forward_gru(self, z, target_point):
    z = self.join(z)  # [B, 512] -> [B, 64]
    
    output_wp = list()
    x = torch.zeros(size=(z.shape[0], 2), dtype=z.dtype).to(z.device)
    
    target_point = target_point.clone()
    target_point[:, 1] *= -1
    
    # Autoregressive generation of 4 waypoints
    for _ in range(self.pred_len):  # pred_len = 4
        if self.gru_concat_target_point:
            x_in = torch.cat([x, target_point], dim=1)  # [B, 4]
        else:
            x_in = x  # [B, 2]
        
        z = self.decoder(x_in, z)  # GRU cell
        dx = self.output(z)  # [B, 3] -> (dx, dy, brake)
        
        x = dx[:,:2] + x  # Accumulate position
        output_wp.append(x[:,:2])
    
    pred_wp = torch.stack(output_wp, dim=1)  # [B, 4, 2]
    
    # Convert from vehicle to lidar coordinate
    pred_wp[:, :, 0] = pred_wp[:, :, 0] - self.config.lidar_pos[0]
    
    return pred_wp, None, None, None, None
```

---

## **STEP 8: MPC Controller - `control_mpc()`**

**File:** `model.py`  
**Lines:** 707-773

```python
def control_mpc(self, waypoints, velocity, is_stuck):
    # Handle input formats
    if isinstance(waypoints, torch.Tensor):
        if waypoints.dim() == 3:
            waypoints = waypoints[0].data.cpu().numpy()  # [1, N, 2] -> [N, 2]
    
    # Transform from lidar to vehicle coordinate
    waypoints[:, 0] += self.config.lidar_pos[0]
    
    # Extract speed
    if isinstance(velocity, torch.Tensor):
        speed = velocity[0].data.cpu().numpy()
    else:
        speed = float(velocity)
    
    # Compute desired speed from waypoint spacing
    if is_stuck:
        desired_speed = self.config.default_speed
    else:
        if len(waypoints) > 1:
            desired_speed = np.linalg.norm(waypoints[1] - waypoints[0]) / self.config.mpc_dt
            desired_speed = np.clip(desired_speed, 0.0, self.config.mpc_max_speed)
        else:
            desired_speed = self.config.default_speed
    
    # Check brake condition
    brake = ((desired_speed < self.config.brake_speed) or 
             ((speed / desired_speed) > self.config.brake_ratio) 
             if desired_speed > 0.01 else False)
    
    # Run MPC optimization
    if brake:
        steer, throttle = self._mpc_optimize(waypoints, speed, desired_speed, brake=True)
        return steer, 0.0, True
    else:
        steer, throttle = self._mpc_optimize(waypoints, speed, desired_speed, brake=False)
        steer = np.clip(steer, -1.0, 1.0)
        throttle = np.clip(throttle, 0.0, self.config.clip_throttle)
        return steer, throttle, False
```

---

## **STEP 9: MPC Optimization - `_mpc_casadi()`**

**File:** `model.py`  
**Lines:** 804-907

```python
def _mpc_casadi(self, waypoints, current_speed, desired_speed, brake, ...):
    N = len(waypoints)  # Horizon = 4
    
    # Create optimization problem
    opti = ca.Opti()
    
    # Decision variables
    X = opti.variable(4, N+1)  # State: [x, y, yaw, speed]
    U = opti.variable(2, N)    # Control: [steer, throttle]
    
    # Initial state (vehicle frame, starting at origin)
    x0 = ca.DM([0.0, 0.0, 0.0, current_speed])
    opti.subject_to(X[:, 0] == x0)
    
    cost = 0
    
    # Dynamics constraints for each step
    for k in range(N):
        x_k = X[0, k]
        y_k = X[1, k]
        yaw_k = X[2, k]
        speed_k = X[3, k]
        
        steer_k = U[0, k]
        throttle_k = U[1, k]
        
        # Kinematic bicycle model
        wheel = steer_gain * steer_k
        beta = ca.atan(rear_wb / (front_wb + rear_wb) * ca.tan(wheel))
        
        if brake:
            accel = brake_accel
        else:
            accel = throt_accel * throttle_k
        
        # State derivatives
        x_dot = speed_k * ca.cos(yaw_k + beta)
        y_dot = speed_k * ca.sin(yaw_k + beta)
        yaw_dot = speed_k / rear_wb * ca.sin(beta)
        speed_dot = accel
        
        # Euler integration
        x_next = x_k + x_dot * dt
        y_next = y_k + y_dot * dt
        yaw_next = yaw_k + yaw_dot * dt
        speed_next = ca.fmax(0.0, speed_k + speed_dot * dt)
        
        opti.subject_to(X[0, k+1] == x_next)
        opti.subject_to(X[1, k+1] == y_next)
        opti.subject_to(X[2, k+1] == yaw_next)
        opti.subject_to(X[3, k+1] == speed_next)
        
        # COST FUNCTION
        # Tracking error
        wp = waypoints[k]
        tracking_error = (X[0, k+1] - wp[0])**2 + (X[1, k+1] - wp[1])**2
        cost += self.config.mpc_q_tracking * tracking_error
        
        # Speed tracking
        speed_error = (X[3, k+1] - desired_speed)**2
        cost += self.config.mpc_q_speed * speed_error
        
        # Control effort
        cost += self.config.mpc_r_steer * steer_k**2
        cost += self.config.mpc_r_throttle * throttle_k**2
    
    # Terminal cost
    if N > 0:
        wp_terminal = waypoints[-1]
        terminal_error = (X[0, N] - wp_terminal[0])**2 + (X[1, N] - wp_terminal[1])**2
        cost += self.config.mpc_q_terminal * terminal_error
    
    # CONSTRAINTS
    opti.subject_to(opti.bounded(-1.0, U[0, :], 1.0))  # Steering
    opti.subject_to(opti.bounded(0.0, U[1, :], self.config.clip_throttle))  # Throttle
    opti.subject_to(opti.bounded(self.config.mpc_min_speed, X[3, :], 
                                 self.config.mpc_max_speed))  # Speed
    
    # SOLVE
    opti.minimize(cost)
    opti.solver('ipopt', {'print_level': 0})
    
    try:
        sol = opti.solve()
        steer = float(sol.value(U[0, 0]))  # First control action
        throttle = float(sol.value(U[1, 0]))
    except:
        # Fallback if optimization fails
        if len(waypoints) > 0:
            aim = waypoints[0]
            angle = np.arctan2(aim[1], aim[0])
            steer = np.clip(angle / (np.pi/2), -1.0, 1.0)
        else:
            steer = 0.0
        throttle = 0.5 if not brake else 0.0
    
    return steer, throttle
```

---

## **STEP 10: Safety Checks & Final Control**

**File:** `submission_agent.py`  
**Lines:** 368-398

```python
# After MPC returns steer, throttle, brake

# Stuck handling
if is_stuck and self.forced_move==1:
    steer = 0.0  # No steering when first unblocking

# Steer damping when braking
if brake or is_stuck:
    steer *= self.steer_damping  # Reduces steering by 0.5x

# Update stuck detector
if(gt_velocity < 0.1):
    self.stuck_detector += 1
elif(gt_velocity > 0.1 and is_stuck == False):
    self.stuck_detector = 0
    self.forced_move = 0

# Create control command
control = carla.VehicleControl()
control.steer = float(steer)
control.throttle = float(throttle)
control.brake = float(brake)

# Emergency braking check
if self.use_lidar_safe_check:
    emergency_stop = (len(safety_box) > 0)  # Obstacle in front?
    if ((emergency_stop == True) and (is_stuck == True)):
        print("Detected object directly in front. Stopping.")
        control.steer = float(steer)
        control.throttle = float(0.0)
        control.brake = float(True)

# Update GPS buffer for next frame
self.update_gps_buffer(self.control, tick_data['compass'], tick_data['speed'])

return control  # Send to CARLA
```

---

## **Data Flow Summary**

```
CARLA input_data
    ↓
tick() → {rgb, lidar, gps, speed, compass, target_point}
    ↓
prepare_image() → [1, 3, 160, 704]
prepare_lidar() → [1, 2, 256, 256]
prepare_goal_location() → target_point [1, 2], target_image [1, 1, 256, 256]
    ↓
forward_ego() → Neural network
    ├─ TransFuser backbone → fused_features [B, 512]
    ├─ forward_gru() → pred_wp [B, 4, 2]
    └─ CenterNet head → bboxes [N, 8]
    ↓
control_mpc(pred_wp, velocity, is_stuck)
    ├─ _mpc_optimize() → MPC solver
    └─ Returns: steer, throttle, brake
    ↓
Safety checks → Final control
    ↓
carla.VehicleControl → CARLA simulator
```

---

## **Key Variables at Each Step**

| Step | Variable | Shape/Type | Location |
|------|----------|------------|----------|
| 1 | `input_data` | CARLA sensor dict | `run_step()` |
| 2 | `tick_data` | `{'rgb', 'lidar', 'gps', 'speed', 'compass', 'target_point'}` | `tick()` |
| 3 | `image` | `[1, 3, 160, 704]` | `prepare_image()` |
| 4 | `lidar_bev` | `[1, 2, 256, 256]` | `prepare_lidar()` |
| 5 | `target_point` | `[1, 2]` | `prepare_goal_location()` |
| 6 | `fused_features` | `[1, 512]` | `forward_ego()` → `_model()` |
| 7 | `pred_wp` | `[1, 4, 2]` | `forward_gru()` |
| 8 | `steer, throttle, brake` | `float, float, bool` | `control_mpc()` |
| 9 | `control` | `carla.VehicleControl` | `run_step()` |

---

This shows the exact code path from sensor input to control output!

