# HIT LiDAR Data Pipeline Report

> System: HIT_Integrated_test (`/home/sentry/AstarTraining/RM_Sentry_2026/HIT_Integrated_test/`)
> Launch: `HIT_intergration_test.launch`
> Robot: New robot, single MID360, upright LiDAR, 20° physical tilt

## Pipeline Overview (Current — 2026-03-21)

> Following Old_nav pipeline strictly: driver → hdl_localization direct (no filter/merger).

```
Livox MID360 (192.168.1.107)
  │  rviz_MID360.launch → MID360_config.json
  │  xfer_format=0 (PointCloud2), roll=20°, msg_frame_id=aft_mapped, rviz_enable=false
  ├── /livox/lidar (PointCloud2, frame: aft_mapped)
  └── /livox/imu   (Imu, raw sensor frame)
        │
        ▼
    imu_filter (bot_sim)
        │  ×(-gravity) on all 3 accel axes (FIXED — was wrong sign on X)
        │  -20° X-axis rotation (FIXED — was missing)
        ▼
    /livox/imu_filtered (Imu, frame: aft_mapped via output_frame_id)
        │
        ├───────────────────┐
        ▼                   ▼
  hdl_localization     [Point-LIO SLAM]
  (NDT-OMP + UKF)     (separate launch only)
  sub: /livox/lidar    sub: /livox/lidar
  (PointCloud2 direct)
  sub: /livox/imu_filtered
        │
        ├─ /odom (Odometry, frame: map)
        ├─ /aligned_points (PointCloud2, frame: map)
        └─ TF: map → aft_mapped
              │
              ▼
        real_robot_transform (bot_sim)
        TF: aft_mapped → gimbal_frame
        translation: (0,0,0) (FIXED)
        rotation: inverse(roll,pitch) (FIXED)
              │
              ▼
        local_frame_bridge.py (passthrough=true)
        /odom → /odom_local
        /aligned_points → /aligned_points_local
              │
              ├─ trajectory_generation (global A*)
              ├─ tracking_node (local MPC)
              └─ hit_bridge → /cmd_vel → mcu_communicator
```

**Disabled nodes:**
- `livox_cloudpoint_processor` (step 6) — expects CustomMsg input, incompatible with PointCloud2 driver
- `map_server` — unless `enable_map_server=true`
- `dstarlite` — unless `enable_dstarlite=true`

## Previous Pipeline (DEPRECATED — Cause 7 in UPSIDE_DOWN report)

The previous configuration used `livox_cloudpoint_processor` (ws_cloud) to filter
the pointcloud and published `/filted_topic_3d` → hdl_localization. This required
`xfer_format=1` (CustomMsg) from the driver, which broke hdl_localization's direct
subscription. The filter is now disabled; hdl_localization receives raw PointCloud2
directly from the driver, matching Old_nav exactly.

## Topic Inventory

### Active Topics

| Topic | Type | Frame | Publisher | Subscriber(s) | Status |
|-------|------|-------|-----------|---------------|--------|
| `/livox/lidar` | PointCloud2 | `livox_frame` | livox_ros_driver2 | hdl_localization | ✅ ACTIVE |
| `/livox/imu` | Imu | (sensor) | livox_ros_driver2 | imu_filter | ✅ ACTIVE |
| `/livox/imu_filtered` | Imu | `aft_mapped` | imu_filter | hdl_localization | ✅ ACTIVE |
| `/odom` | Odometry | `map` | hdl_localization | local_frame_bridge | ✅ ACTIVE |
| `/aligned_points` | PointCloud2 | `map` | hdl_localization | local_frame_bridge | ✅ ACTIVE |
| `/odom_local` | Odometry | `map` | local_frame_bridge | trajectory_gen, tracking_node | ✅ ACTIVE |
| `/aligned_points_local` | PointCloud2 | `map` | local_frame_bridge | (unused passthrough) | ⚠️ Published but unused |
| `/plan` | Path | `map` | trajectory_generation | tracking_node | ✅ ACTIVE |
| `/sentry_des_speed` | slaver_speed | — | tracking_node | hit_bridge | ✅ ACTIVE |
| `/cmd_vel` | Twist | — | hit_bridge | mcu_communicator | ✅ ACTIVE |
| `/dstar_status` | Bool | — | hit_bridge | mcu_communicator | ✅ ACTIVE |
| `/clicked_point` | PointStamped | `map` | local_frame_bridge | trajectory_generation | ✅ ACTIVE |

### Disabled Topics

| Topic | Would-be Publisher | Status |
|-------|--------------------|--------|
| `/3Dlidar` | livox_cloudpoint_processor | ❌ Disabled (processor off) |
| `/filted_topic_3d` | livox_cloudpoint_processor | ❌ Disabled (processor off) |
| `/grid` | livox_cloudpoint_processor | ❌ Disabled (processor off) |
| `/test_scan_2d` | livox_cloudpoint_processor | ❌ Disabled (processor off) |
| `/map` (OccupancyGrid) | map_server | ❌ Disabled unless `enable_map_server=true` |
| D*Lite topics | dstarlite | ❌ Disabled unless `enable_dstarlite=true` |
| ser2msg topics | ser2msg_tf_decision | ❌ Disabled unless `enable_ser2msg=true` |

### Bot_sim Topics (NOT USED in HIT pipeline)

| Topic | Would-be Publisher | Why Unused |
|-------|--------------------|------------|
| `/3Dlidar` (via bot_sim merge) | threeD_lidar_merge_pointcloud | Launch uses livox_cloudpoint_processor instead |
| `/filted_topic_3d` (via bot_sim filter) | threeD_lidar_filter_pointcloud | Launch uses livox_cloudpoint_processor instead |

## Key Differences from Old_nav

| Aspect | Old_nav | HIT (current) |
|--------|---------|---------------|
| LiDAR filter | DISABLED | DISABLED (same) |
| hdl_localization input | `/livox/lidar` (PointCloud2 direct) | `/livox/lidar` (PointCloud2 direct, same) |
| Livox driver launch | `rviz_MID360.launch` | `rviz_MID360.launch` (same, rviz_enable=false) |
| Driver config | `MID360_config.json` (roll=20°) | `MID360_config.json` (roll=20°, same) |
| Planning stack | D*Lite + ser2msg | HIT A* + MPC + hit_bridge |
| Output to MCU | ser2msg (direct) | mcu_communicator (0x93 frame) |

## Source Files

| File | Package | Workspace | Used? |
|------|---------|-----------|-------|
| `livox_cloudpoints_processor.cpp` | livox_cloudpoint_processor | ws_cloud | ✅ Yes (THE main filter) |
| `imu_filter.cpp` | bot_sim | sim_nav | ✅ Yes |
| `real_robot_transform.cpp` | bot_sim | sim_nav | ✅ Yes |
| `local_frame_bridge.py` | bot_sim | sim_nav | ✅ Yes |
| `threeD_lidar_merge_pointcloud.cpp` | bot_sim | sim_nav | ❌ Not used |
| `threeD_lidar_filter_pointcloud.cpp` | bot_sim | sim_nav | ❌ Not used |
