> **⚠️ HISTORICAL** — Documents the superseded `Old_nav/` pipeline.  For the current HIT system, see [HIT_COORDINATE_SYSTEMS_REPORT.md](HIT_COORDINATE_SYSTEMS_REPORT.md).

# Old_nav Coordinate Systems Report

> System: Old_nav (`/home/sentry/AstarTraining/Old_nav/`)
> Launch: `3DNavUL_Test.launch`

## TF Tree

```
map
 └── aft_mapped        (hdl_localization: NDT-computed, ~5 Hz)
      └── gimbal_frame  (real_robot_transform: roll/pitch-leveled, 20 Hz)
```

## Coordinate Frame Definitions

### `map` (Global/World Frame)
- **Origin**: Defined by the globalmap PCD (current.pcd from Point-LIO mapping run)
- **Orientation**: Z-up, gravity-aligned (from Point-LIO IMU integration)
- **Published by**: hdl_localization (as parent of `aft_mapped`)
- **Topics in this frame**: `/odom`, `/aligned_points`

### `aft_mapped` (LiDAR Odometry Frame)
- **Origin**: Moves with the robot; position in `map` is computed by NDT scan matching
- **Orientation**: Aligned with the LiDAR sensor, but with 20° tilt compensation from driver
- **Published by**: Livox driver sets frame_id, hdl_localization publishes TF from `map`
- **Topics in this frame**: `/livox/lidar`, `/livox/imu_filtered`
- **Note**: The Livox driver applies roll=20° extrinsic rotation, so data in this frame is approximately level

### `gimbal_frame` (Robot Body Frame)
- **Origin**: Co-located with `aft_mapped` (translation = 0,0,0)
- **Orientation**: Roll and pitch stripped from `aft_mapped`, so it's always level (yaw follows robot heading)
- **Published by**: real_robot_transform node
- **Computation**: Looks up `aft_mapped → map`, extracts roll/pitch, inverts, applies as `aft_mapped → gimbal_frame`
- **Used by**: Navigation, obstacle avoidance (level reference frame)

## Coordinate Transformations Applied

### 1. Livox Driver → `/livox/lidar`
- **Input**: Raw LiDAR points in sensor frame (20° tilted)
- **Transform**: Extrinsic rotation roll=20° (in `MID360_config.json`)
- **Output**: Points in `aft_mapped` frame (approximately level)
- **Equation**: `p_out = R_x(20°) × p_raw`

### 2. Livox Driver → `/livox/imu`
- **Input**: Raw IMU data from sensor
- **Transform**: None (raw from driver)
- **Output**: Raw IMU in sensor frame (20° tilted)

### 3. imu_filter: `/livox/imu` → `/livox/imu_filtered`
- **Transform**:
  1. Scale: `accel *= -gravity` (all 3 axes; converts from g-units to m/s², flips sign for ROS convention)
  2. Rotate: -20° around X-axis for both linear_acceleration and angular_velocity
- **Equation**:
  ```
  a_y' = a_y × cos(-20°) + a_z × sin(-20°)
  a_z' = -a_y × sin(-20°) + a_z × cos(-20°)
  ```
- **Purpose**: Makes IMU data consistent with the driver's 20° tilt compensation

### 4. hdl_localization: NDT Scan Matching
- **Input**: `/livox/lidar` (aft_mapped) + `/livox/imu_filtered` + globalmap PCD
- **Output**: TF `map → aft_mapped` (robot pose in world)
- **Method**: NDT-OMP with UKF prediction from IMU

### 5. real_robot_transform: `aft_mapped → gimbal_frame`
- **Input**: TF `aft_mapped → map`
- **Transform**:
  1. Extract roll, pitch (zero yaw) from the TF rotation
  2. Invert the quaternion: `q = inverse(quat(roll, pitch, 0))`
  3. Apply sensor_offset_q (all zeros → identity)
  4. Result: `q_final = q_inverse × identity = q_inverse`
- **Output**: TF `aft_mapped → gimbal_frame` with zero translation, leveling rotation
- **Effect**: gimbal_frame is always level (parallel to ground), even if robot tilts
