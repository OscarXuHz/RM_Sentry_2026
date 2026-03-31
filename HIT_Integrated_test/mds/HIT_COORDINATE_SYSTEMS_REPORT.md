# HIT Coordinate Systems Report

> System: HIT_Integrated_test (`/home/sentry/AstarTraining/RM_Sentry_2026/HIT_Integrated_test/`)
> Launch: `HIT_intergration_test.launch`

## TF Tree

```
map
 └── aft_mapped          (hdl_localization: NDT-computed, ~5 Hz)
      └── gimbal_frame    (real_robot_transform: roll/pitch-leveled, 20 Hz)
```

## Coordinate Frame Definitions

### `map` (Global/World Frame)
- **Origin**: Defined by globalmap PCD (`pcd/current.pcd`, from Point-LIO mapping run)
- **Orientation**: Z-up, gravity-aligned
- **Published by**: hdl_localization (parent of `aft_mapped`)
- **Topics**: `/odom` (Odometry), `/aligned_points` (PointCloud2)
- **Init pose**: (0,0,0) position, (w=0.7017, z=-0.7017) ≈ -90° yaw

### `aft_mapped` (LiDAR Odometry Frame)
- **Origin**: Moves with robot
- **Orientation**: LiDAR sensor frame, with 20° tilt compensation from driver (FIXED — was missing)
- **Published by**: Livox driver (frame_id), hdl_localization (TF)
- **Topics**: `/livox/lidar`, `/3Dlidar`, `/filted_topic_3d`, `/grid`, `/livox/imu_filtered`

### `livox_frame` (Driver Default Frame)
- **Note**: The Livox driver uses `livox_frame` as default frame_id in `msg_mixed.launch`
- **Relationship**: Functionally equivalent to `aft_mapped` after extrinsic rotation
- **Topics**: Raw CustomMsg before frame_id override

### `gimbal_frame` (Robot Body Frame)
- **Origin**: Co-located with `aft_mapped` (translation = 0,0,0) — FIXED (was -0.011, -0.17166, 0)
- **Orientation**: Level (roll/pitch compensated) — FIXED (was identity/no compensation)
- **Published by**: real_robot_transform node (20 Hz)
- **Used by**: livox_cloudpoint_processor `base_frame` (for output reference)

## Coordinate Transformations Applied

### 1. Livox Driver → `/livox/lidar`
- **Config**: `mixed_HAP_MID360_config.json`
- **Extrinsic**: roll=20.0° (FIXED — was 0.0°), pitch=0, yaw=0
- **Both entries** (HAP slot + MID360 slot) point to same IP 192.168.1.107
- **Effect**: Raw tilted points → level points in driver output

### 2. imu_filter: `/livox/imu` → `/livox/imu_filtered`
- **Transform** (FIXED — synced from Old_nav):
  1. Scale: `accel *= -gravity` (all 3 axes)
  2. Rotate: -20° around X-axis on both accel and gyro
- **Was (broken)**: `accel.x *= gravity` (positive, not negated), no rotation, manual gyro y/z flip
- **Now (fixed)**: Consistent with driver's 20° extrinsic rotation

### 3. livox_cloudpoint_processor: `/livox/lidar` → `/filted_topic_3d`, `/3Dlidar`, `/grid`
- **Point offset** (FIXED — was Y/Z negated for upside-down lidar):
  ```
  x = raw.x - 0.011      (was: x = raw.x - 0.011)      — unchanged
  y = raw.y + 0.02329     (was: y = -raw.y - 0.02329)   — FIXED: removed negation
  z = raw.z - 0.04412     (was: z = -raw.z + 0.04412)   — FIXED: removed negation
  ```
- **ispoint filter** (FIXED):
  ```
  ispoint(x, y, z, ...)   (was: ispoint(x, y, -z, ...)) — FIXED: removed extra negation
  ```
- **Processing pipeline**: radius filter → ispoint height filter → voxel downsample → statistical outlier removal → normal estimation → keep vertical-normal points → 2D grid → dilation → visibility

### 4. hdl_localization: NDT Scan Matching
- **Input**: `/filted_topic_3d` (aft_mapped) + `/livox/imu_filtered`
- **Output**: TF `map → aft_mapped`, `/odom` (map frame)
- **Config**: NDT_OMP, resolution 1.0, downsample 0.1
- **IMU**: `invert_imu_acc=false` (correct for upright lidar)

### 5. real_robot_transform: `aft_mapped → gimbal_frame` (FIXED — synced from Old_nav)
- **Was (broken)**:
  - Translation: (-0.011, -0.17166, 0) — old robot offset
  - Rotation: Identity (commented out) — no tilt compensation
  - `qhdl` NOT inverted
- **Now (fixed)**:
  - Translation: (0, 0, 0) — lidar at gimbal center on new robot
  - Rotation: `inverse(roll, pitch, 0)` × `sensor_offset_q` — levels gimbal_frame
  - `qhdl` properly inverted

### 6. local_frame_bridge.py: Frame Passthrough
- **Mode**: `passthrough=true` (no coordinate shifting)
- **Input**: `/odom` → `/odom_local`, `/filted_topic_3d` → `/filted_topic_3d_local`
- **No transform applied**: Just relays with same frame_ids

## Comparison of Pre-Fix vs Post-Fix

| Component | Pre-Fix (broken) | Post-Fix (synced with Old_nav) |
|-----------|------------------|-------------------------------|
| Driver roll | 0.0° | 20.0° |
| imu_filter accel.x sign | positive (×gravity) | negative (×-gravity) |
| imu_filter rotation | None | -20° X-axis |
| Processor Y | -raw.y - 0.02329 | raw.y + 0.02329 |
| Processor Z | -raw.z + 0.04412 | raw.z - 0.04412 |
| ispoint Z arg | -z | z |
| real_robot_transform translation | (-0.011, -0.17166, 0) | (0, 0, 0) |
| real_robot_transform rotation | Identity (commented out) | Inverse roll/pitch leveling |
| real_robot_transform qhdl | NOT inverted | Inverted |
