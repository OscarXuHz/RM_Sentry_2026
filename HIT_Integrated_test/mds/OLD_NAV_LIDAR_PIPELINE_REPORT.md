> **⚠️ HISTORICAL** — Documents the superseded `Old_nav/` LiDAR pipeline.  For the current HIT system, see [HIT_LIDAR_PIPELINE_REPORT.md](HIT_LIDAR_PIPELINE_REPORT.md).

# Old_nav LiDAR Data Pipeline Report

> System: Old_nav (`/home/sentry/AstarTraining/Old_nav/`)
> Launch: `3DNavUL_Test.launch`
> Robot: New robot, single MID360, upright LiDAR, 20° physical tilt

## Pipeline Overview

```
Livox MID360 (192.168.1.107)
  │  roll=20° in driver config (MID360_config.json)
  │  xfer_format=0 → publishes sensor_msgs::PointCloud2
  │  → compensates physical 20° tilt at driver level
  ├── /livox/lidar  (PointCloud2, frame: aft_mapped)
  └── /livox/imu    (Imu, raw sensor frame)
        │
        ▼
    imu_filter (bot_sim)
        │  ×(-gravity) on all 3 accel axes
        │  -20° X-axis rotation on accel + gyro
        ▼
    /livox/imu_filtered (Imu, frame: aft_mapped)
        │
        ├───────────────────┐
        ▼                   ▼
  hdl_localization     [Point-LIO SLAM]
  (NDT-OMP + UKF)     (separate launch only)
  sub: /livox/lidar    sub: /livox/lidar
  sub: /livox/imu_filtered
        │
        ├─ pub: /odom (Odometry, frame: map)
        ├─ pub: /aligned_points (PointCloud2, frame: map)
        └─ TF: map → aft_mapped
              │
              ▼
        real_robot_transform (bot_sim)
        TF: aft_mapped → gimbal_frame
        translation: (0, 0, 0)
        rotation: inverse(roll,pitch) from TF
```

## Topic Inventory

### Active Topics (used in 3DNavUL_Test.launch)

| Topic | Type | Frame | Publisher | Subscriber(s) | Status |
|-------|------|-------|-----------|---------------|--------|
| `/livox/lidar` | CustomMsg | `aft_mapped` | livox_ros_driver2 | hdl_localization | ✅ ACTIVE |
| `/livox/imu` | Imu | (sensor) | livox_ros_driver2 | imu_filter | ✅ ACTIVE |
| `/livox/imu_filtered` | Imu | `aft_mapped` | imu_filter | hdl_localization | ✅ ACTIVE |
| `/odom` | Odometry | `map` | hdl_localization | ser2msg, decision | ✅ ACTIVE |
| `/aligned_points` | PointCloud2 | `map` | hdl_localization | (debug/viz) | ✅ ACTIVE |

### Disabled/Unused Topics

| Topic | Type | Frame | Would-be Publisher | Status |
|-------|------|-------|--------------------|--------|
| `/3Dlidar` | PointCloud2 | `aft_mapped` | threeD_lidar_merge_pointcloud | ❌ DISABLED (commented out in launch) |
| `/filted_topic_3d` | PointCloud2 | `gimbal_frame` | threeD_lidar_filter_pointcloud | ❌ DISABLED (commented out in launch) |
| `/pointcloud_converted` | PointCloud2 | `aft_mapped` | pointcloud_converter | ❌ DISABLED |

## Key Design Decisions

1. **Single-lidar handling**: Both `scan_topic_left` and `scan_topic_right` in the merge launch point to `/livox/lidar` — same topic, effectively doubling points (filtered by downsampling)
2. **No merge/filter in nav pipeline**: The merge and filter nodes are BOTH commented out in `3DNavUL_Test.launch`. hdl_localization receives raw `/livox/lidar` directly.
3. **20° tilt compensation**: Applied at the Livox DRIVER level (roll=20.0 in `MID360_config.json`), so `/livox/lidar` is already in a level coordinate frame. The imu_filter applies matching -20° rotation to IMU data.
4. **No Y/Z negation anywhere**: All coordinate transforms use simple translational offsets. No sign flipping.

## Livox Driver Config

- **Launch**: `rviz_MID360.launch`
- **Config**: `MID360_config.json`
- **IP**: 192.168.1.107
- **Extrinsic**: roll=20.0, pitch=0, yaw=0, xyz=(0,0,0)
- **Published topic**: `/livox/lidar` (CustomMsg, frame `aft_mapped`)
- **Published IMU**: `/livox/imu`

## Source Files

| File | Package | Used? |
|------|---------|-------|
| `imu_filter.cpp` | bot_sim | ✅ Yes |
| `real_robot_transform.cpp` | bot_sim | ✅ Yes |
| `threeD_lidar_merge_pointcloud.cpp` | bot_sim | ❌ Disabled |
| `threeD_lidar_filter_pointcloud.cpp` | bot_sim | ❌ Disabled |
| `pointcloud_converter.cpp` | bot_sim | ❌ Disabled |
| `livox_cloudpoints_processor.cpp` | livox_cloudpoint_processor (Navigation-filter-test) | ❌ Not used in 3DNavUL |
