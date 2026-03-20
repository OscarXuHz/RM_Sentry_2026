# NUC Alignment Report — RM_Sentry_2026

> **Date:** 20 March 2026  
> **Source robot NUC:** `192.168.1.50` (dual MID360, per-IP topics)  
> **Target NUC (this machine):** `192.168.1.5` (HAP + MID360 mixed, single topics)  
> **Reference:** `/home/sentry/AstarTraining/Old_nav/` (known-working config on this NUC)

---

## Summary

RM_Sentry_2026 was cloned from a robot with a **different LiDAR topology** (dual MID360 at `192.168.1.3` + `192.168.1.105`) and NUC IP (`192.168.1.50`). This NUC runs a **mixed HAP + MID360** setup at `192.168.1.107` with host IP `192.168.1.5`.

**Total files modified:** 12  
**Build status:** All 5 workspaces built successfully  
**Build fix:** 1 missing source file (`chase.cpp`) copied from workspace DecisionNode  

---

## 1. LiDAR Hardware Configuration Changes

### 1.1 Livox Driver Config — `MID360_config.json`
**File:** `ws_livox/src/livox_ros_driver2/config/MID360_config.json`

| Parameter | Old (source robot) | New (this NUC) |
|-----------|-------------------|----------------|
| `host_net_info.cmd_data_ip` | 192.168.1.50 | **192.168.1.5** |
| `host_net_info.push_msg_ip` | 192.168.1.50 | **192.168.1.5** |
| `host_net_info.point_data_ip` | 192.168.1.50 | **192.168.1.5** |
| `host_net_info.imu_data_ip` | 192.168.1.50 | **192.168.1.5** |
| `lidar_configs[0].ip` | 192.168.1.105 | **192.168.1.107** |
| `lidar_configs[0].extrinsic.roll` | 0.0 | **20.0** (gimbal tilt compensation) |

### 1.2 Mixed HAP+MID360 Config — `mixed_HAP_MID360_config.json`
**File:** `ws_livox/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json`

**Replaced entirely.** Was a dual-MID360 config (IPs `192.168.1.3` + `192.168.1.105`, host `192.168.1.50`, y-offset 390mm between units). Now a proper HAP + MID360 mixed config:

| Section | Value |
|---------|-------|
| HAP host IP | 192.168.1.5 |
| HAP ports | 56000/57000/58000/59000 |
| MID360 host IP | 192.168.1.5 |
| MID360 ports | 56100–56500 |
| Both LiDAR IPs | 192.168.1.107 |

### 1.3 Driver Launch — `msg_mixed.launch`
**File:** `ws_livox/src/livox_ros_driver2/launch_ROS1/msg_mixed.launch`

| Parameter | Old | New |
|-----------|-----|-----|
| `multi_topic` | 1 (per-IP topics) | **0** (single combined topic) |
| `msg_frame_id` | `aft_mapped` | **`livox_frame`** |
| `user_config_path` | `dual_MID360_config.json` | **`mixed_HAP_MID360_config.json`** |

**Impact:** Topic names change from `/livox/lidar_192_168_1_X` and `/livox/imu_192_168_1_X` to `/livox/lidar` and `/livox/imu`.

---

## 2. Topic Remapping — Downstream Subscribers

All downstream nodes that subscribed to per-IP LiDAR/IMU topics were updated to use the single combined topics.

### 2.1 IMU Filter — `imu_filter.launch`
**File:** `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/imu_filter.launch`

| Old (dual IMU) | New (single IMU) |
|----------------|------------------|
| Node `imu_filter_105`: `/livox/imu_192_168_1_105` → `..._filtered` | **Removed** |
| Node `imu_filter_3`: `/livox/imu_192_168_1_3` → `..._filtered` | **Replaced** with single node |
| — | Node `imu_filter`: `/livox/imu` → `/livox/imu_filtered` |

### 2.2 LiDAR Merge — `lidar_merge_pointcloud.launch`
**File:** `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch`

| Parameter | Old | New |
|-----------|-----|-----|
| `scan_topic_left` | `/livox/lidar_192_168_1_3` | **`/livox/lidar`** |
| `scan_topic_right` | `/livox/lidar_192_168_1_105` | **`/livox/lidar`** |
| `max_height` | (missing) | **10.0** |
| `min_height` | (missing) | **-1.0** |

### 2.3 Cloud Point Processor — `lidar_filter_pointcloud.launch`
**File:** `ws_cloud/src/livox_cloudpoint_processor/launch/lidar_filter_pointcloud.launch`

| Parameter | Old | New |
|-----------|-----|-----|
| `scan_topic_left` | `/livox/lidar_192_168_1_105` | **`/livox/lidar`** |
| `scan_topic_right` | `/livox/lidar_192_168_1_105` | **`/livox/lidar`** |

### 2.4 Point-LIO SLAM — `avia.yaml`
**File:** `HIT_Integrated_test/sim_nav/src/Point-LIO/config/avia.yaml`

| Parameter | Old | New |
|-----------|-----|-----|
| `lid_topic` | `/livox/lidar_192_168_1_3` | **`/livox/lidar`** |
| `imu_topic` | `/livox/imu_192_168_1_3` | **`/livox/imu`** |

### 2.5 HDL Localization — `hdl_localization.launch`
**File:** `HIT_Integrated_test/sim_nav/src/hdl_localization/launch/hdl_localization.launch`

| Parameter | Old | New |
|-----------|-----|-----|
| `points_topic` (default) | `/3Dlidar` | **`/livox/lidar`** |
| `imu_topic` | `/livox/imu_192_168_1_3_filtered` | **`/livox/imu_filtered`** |
| `invert_imu_acc` | `false` | **`true`** |
| `globalmap_pcd` | `/home/sentry_train_test/.../current.pcd` | **`$(find hdl_graph_slam)/map/Mar16.pcd`** |
| `init_pos_z` | 0.3 | **0.0** |

### 2.6 Robot Transform — `real_robot_transform.launch`
**File:** `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/real_robot_transform.launch`

| Change | Detail |
|--------|--------|
| Added `roll_offset_deg`, `pitch_offset_deg`, `yaw_offset_deg` params (all 0.0) | Matches Old_nav format |
| Removed `base_link` static TF publisher | Not present in Old_nav config |

---

## 3. NUC Path Fixes

All hardcoded paths from the source robot's home directory were updated.

### 3.1 Master Launch — `HIT_intergration_test.launch`
**File:** `HIT_Integrated_test/HIT_intergration_test.launch`

| Path element | Old | New |
|-------------|-----|-----|
| `ROS_PACKAGE_PATH` | `/home/sentry_train_test/AstarTraining/DecisionNode/...` | **`/home/sentry/AstarTraining/RM_Sentry_2026/DecisionNode/...`** |
| `ROS_PACKAGE_PATH` | `/home/sentry_train_test/AstarTraining/HIT_code/...` | **`/home/sentry/AstarTraining/RM_Sentry_2026/HIT_code/...`** |
| `CMAKE_PREFIX_PATH` | `/home/sentry_train_test/AstarTraining/DecisionNode/...` | **`/home/sentry/AstarTraining/RM_Sentry_2026/DecisionNode/...`** |

### 3.2 Map Builder Launch — `HIT_pointlio_map_builder.launch`
**File:** `HIT_Integrated_test/HIT_pointlio_map_builder.launch`

All 5 occurrences of `/home/sentry_train_test/AstarTraining/HIT_Integrated_test/` replaced with `/home/sentry/AstarTraining/RM_Sentry_2026/HIT_Integrated_test/`.

---

## 4. Missing Files Resolved

| File | Source | Destination | Reason |
|------|--------|-------------|--------|
| `chase.cpp` | `AstarTraining/DecisionNode/src/decision_node/src/` | `RM_Sentry_2026/DecisionNode/src/decision_node/src/` | Missing from repo sync; needed by `strategy_node` target |
| `Mar16.pcd` | `Old_nav/sim_nav/src/hdl_graph_slam/map/` | `RM_Sentry_2026/HIT_Integrated_test/sim_nav/src/hdl_graph_slam/map/` | Localization reference map; excluded by .gitignore (*.pcd) |
| `pcd/` directory | *(created)* | `RM_Sentry_2026/HIT_Integrated_test/pcd/` | Output directory for Point-LIO map builder |

---

## 5. Build Results

| Workspace | Path | Status | Notes |
|-----------|------|--------|-------|
| ws_livox | `RM_Sentry_2026/ws_livox/` | **OK** | Clean build |
| ws_cloud | `RM_Sentry_2026/ws_cloud/` | **OK** | Clean build |
| sentry_planning_ws | `RM_Sentry_2026/HIT_code/sentry_planning_ws/` | **OK** | Warnings only (unused vars, sign-compare) |
| sim_nav | `RM_Sentry_2026/HIT_Integrated_test/sim_nav/` | **OK** | Warnings only (format-security in hdl_localization) |
| DecisionNode | `RM_Sentry_2026/DecisionNode/` | **OK** | After copying `chase.cpp`; CMake warning about BehaviorTree DEPENDS |

### Package Resolution (verified)
```
livox_ros_driver2       → ws_livox/src/livox_ros_driver2
livox_cloudpoint_processor → ws_cloud/src/livox_cloudpoint_processor
hdl_localization        → sim_nav/src/hdl_localization
point_lio               → sim_nav/src/Point-LIO
trajectory_generation   → sentry_planning_ws/src/.../trajectory_generation
tracking_node           → sentry_planning_ws/src/.../trajectory_tracking
decision_node           → DecisionNode/src/decision_node
hdl_graph_slam          → sim_nav/src/hdl_graph_slam
bot_sim                 → sim_nav/src/bot_sim
```

---

## 6. Network Architecture Comparison

```
Source Robot (192.168.1.50):            This NUC (192.168.1.5):
┌────────────────────────┐              ┌─────────────────────────┐
│  NUC (192.168.1.50)    │              │  NUC (192.168.1.5)      │
│                        │              │                         │
│  ┌──────────────────┐  │              │  ┌───────────────────┐  │
│  │ livox_ros_driver2 │  │              │  │ livox_ros_driver2  │  │
│  │ dual_MID360_cfg  │  │              │  │ mixed_HAP_MID360   │  │
│  │ multi_topic=1    │  │              │  │ multi_topic=0      │  │
│  └──┬──────────┬────┘  │              │  └──────┬────────────┘  │
│     │          │        │              │         │               │
│  /lidar_1.3  /lidar_1.105             │    /livox/lidar          │
│  /imu_1.3    /imu_1.105  │              │    /livox/imu            │
└────┬──────────┬──────────┘              └────────┬───────────────┘
     │          │                                  │
┌────┴────┐ ┌──┴───────┐               ┌──────────┴──────────┐
│MID360   │ │MID360    │               │ HAP + MID360 combo  │
│1.3      │ │1.105     │               │ 192.168.1.107       │
│(mapping)│ │(obstacle)│               │ roll offset: 20°    │
└─────────┘ └──────────┘               └─────────────────────┘
```

---

## 7. Remaining Notes

1. **Serial port:** No serial devices (`/dev/ttyUSB*`, `/dev/ttyACM*`) currently detected. The master launch has `enable_mcu_communicator=true` by default — set to `false` for testing without MCU, or connect the serial board.

2. **PCD map (Mar16.pcd):** Copied from Old_nav. If updating to a new arena, re-map with:
   ```bash
   source hit_env.bash
   roslaunch HIT_pointlio_map_builder.launch
   # After map is built, rename and convert:
   cd RM_Sentry_2026/HIT_Integrated_test/pcd && mv scans.pcd current.pcd
   python3 RM_Sentry_2026/HIT_code/sentry_planning_ws/tools/pcd_to_maps.py current.pcd
   ```

3. **`invert_imu_acc`:** Set to `true` for this NUC (matches Old_nav). The source robot had it `false`, likely due to different IMU orientation on the dual-MID360 bracket.

4. **MID360 roll offset (20°):** The `MID360_config.json` has `roll: 20.0` for the gimbal-mounted MID360. This extrinsic is applied in the driver before publishing. If the physical mount changes, update this value.

5. **`hit_env.bash` uses relative paths** — no path fix needed; it auto-resolves from its script location.
