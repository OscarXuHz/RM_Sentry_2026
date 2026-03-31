> **⚠️ HISTORICAL** — The ws_cloud processing path this fixes is not used by the current HIT pipeline (driver → hdl_localization direct).  Keep for reference if ws_cloud processing is re-enabled.

# Upside-Down /3DLidar Pointcloud Fix Report

## Symptom
The `/3Dlidar` merged pointcloud and `/filted_topic_3d` filtered pointcloud appeared **upside-down** in RViz, even though TF frames (`aft_mapped`, `gimbal_frame`) from Point-LIO SLAM appeared correct.

## Investigation Summary

### Why Point-LIO worked but navigation didn't
Point-LIO subscribes directly to `/livox/lidar` (raw Livox driver output), bypassing all processing nodes. The upside-down inversions only exist in the `livox_cloudpoint_processor` which produces `/filted_topic_3d` and `/3Dlidar`. hdl_localization in HIT was configured to use `/filted_topic_3d` (containing the inverted data), not `/livox/lidar`.

### The previous fix was to the WRONG files
The `bot_sim` package's `threeD_lidar_merge_pointcloud.cpp` and `threeD_lidar_filter_pointcloud.cpp` were edited previously, but **these files are NOT used** in the HIT pipeline. The HIT launch file (`HIT_intergration_test.launch`, step 6) uses:
```xml
<include file="$(find livox_cloudpoint_processor)/launch/lidar_filter_pointcloud.launch" />
```
This resolves to `ws_cloud/src/livox_cloudpoint_processor/`, a completely different package from `bot_sim`.

## Root Causes (5 total)

### Cause 1: Y/Z Negation in livox_cloudpoints_processor.cpp (PRIMARY)
**File**: `ws_cloud/src/livox_cloudpoint_processor/src/livox_cloudpoints_processor.cpp`

The processor had hardcoded `y = -y` and `z = -z` coordinate negation in ALL 4 processing loops (origin left/right, filtered left/right). This was legacy compensation for the old robot's upside-down LiDAR mount.

```cpp
// BROKEN (old upside-down robot):
double y = -scan_record_left.points[i].y - 0.02329;
double z = -scan_record_left.points[i].z + 0.04412;
if (ispoint(x, y, -z, intensity))  // additional Z negation in filter

// FIXED (new upright robot):
double y = scan_record_left.points[i].y + 0.02329;
double z = scan_record_left.points[i].z - 0.04412;
if (ispoint(x, y, z, intensity))   // no negation
```

### Cause 2: Livox Driver Config Missing 20° Tilt Compensation
**File**: `ws_livox/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json`

The LiDAR is physically tilted 20°. Old_nav's `MID360_config.json` has `roll: 20.0` to compensate at the driver level. The HIT config had `roll: 0.0`, meaning the pointcloud was published still tilted.

```json
// BROKEN: "roll": 0.0
// FIXED:  "roll": 20.0
```

### Cause 3: IMU Filter Missing -20° Rotation
**File**: `sim_nav/src/bot_sim/src/imu_filter.cpp`

The HIT version was missing the -20° X-axis rotation that Old_nav applies to make IMU data consistent with the driver's tilt compensation. It also had the wrong sign on X acceleration (`×gravity` instead of `×-gravity`).

**Old HIT (broken):**
- `accel.x *= gravity` (positive — wrong sign)
- No rotation applied
- Manual `gyro.y *= -1; gyro.z *= -1`

**Fixed (synced from Old_nav):**
- `accel.x *= -gravity` (negative — correct)
- -20° X-axis rotation on both accel and gyro
- No manual gyro flipping (rotation handles it)

### Cause 4: real_robot_transform Hardcoded for Old Robot
**File**: `sim_nav/src/bot_sim/src/real_robot_transform.cpp`

The HIT version had old-robot hardcoded values and disabled rotation correction:
- Translation: (-0.011, -0.17166, 0) → should be (0, 0, 0) for new robot
- Rotation application was commented out → should use inverse(roll,pitch) leveling
- `qhdl` was not inverted → should be inverted

**Fixed**: Synced code from Old_nav (parameterized frames, inverse qhdl, zero translation).

### Cause 5: Prior fixes from LOCALIZATION_FIX_REPORT (already applied)
- globalmap PCD: Changed from Mar16.pcd to current.pcd ✅
- Init pose: Changed from arbitrary to origin ✅
- `invert_imu_acc`: Changed from true to false ✅

## Files Modified

| File | Workspace | Change |
|------|-----------|--------|
| `livox_cloudpoints_processor.cpp` | ws_cloud | Removed Y/Z negation in 4 loops, fixed ispoint calls |
| `mixed_HAP_MID360_config.json` | ws_livox | Set roll=20.0 for both lidar entries |
| `imu_filter.cpp` | sim_nav (bot_sim) | Synced from Old_nav: -20° rotation, fixed accel signs |
| `real_robot_transform.cpp` | sim_nav (bot_sim) | Synced from Old_nav: parameterized, inverse qhdl, zero translation |

## Build Verification
- `ws_cloud`: catkin_make → [100%] livox_cloudpoint_processor compiled, zero errors
- `sim_nav` (bot_sim): catkin_make → [100%] imu_filter + real_robot_transform compiled, zero errors

## Cause 6 (2026-03-21): rviz crash on headless NUC

### Symptom
After switching step 3 in `HIT_intergration_test.launch` to `rviz_MID360.launch`, the rviz node crashed: `[livox_rviz-4] process has died [exit code -6]`.

### Root Cause
`rviz_MID360.launch` defaults to `rviz_enable=true`, spawning a GUI rviz process. The NUC runs headless (no X display server) → rviz gets SIGABRT (exit -6) on OpenGL/Qt init.

### Fix
Override `rviz_enable=false` when including the launch.

---

## Cause 7 (2026-03-21): Localization float — wrong pipeline architecture

### Symptom
Pointcloud upright, but localization constantly floats away after switching to `rviz_MID360.launch`.

### Root Cause — Pipeline Mismatch
The Old_nav working pipeline is:
```
Livox driver (xfer_format=0, PointCloud2) → /livox/lidar → hdl_localization (direct)
```
**No filter, no merger.** hdl_localization subscribes to `/livox/lidar` as `sensor_msgs::PointCloud2` directly.

The HIT launch was configured with:
- `points_topic=/filted_topic_3d` → hdl_localization listening to the processor's output
- `livox_cloudpoint_processor` enabled (step 6), expecting `CustomMsg` input
- But `rviz_MID360.launch` uses `xfer_format=0` (PointCloud2)

Result: type mismatch. The processor can't receive PointCloud2 (expects CustomMsg), so `/filted_topic_3d` is never published. hdl_localization gets no scan data → drifts on IMU-only prediction.

**Earlier Cause 6 fix was also wrong:** Setting `xfer_format=1` (CustomMsg) would feed the processor, but then hdl_localization on `/livox/lidar` directly would fail (hdl_localization expects PointCloud2, not CustomMsg). Either way, something breaks.

### Correct Fix — Follow Old_nav Strictly
Three changes to `HIT_intergration_test.launch`:

1. **points_topic**: `/filted_topic_3d` → `/livox/lidar` (direct PointCloud2 from driver)
2. **Driver**: `xfer_format=0` (PointCloud2, the default) + `rviz_enable=false`
3. **Processor disabled**: `livox_cloudpoint_processor` (step 6) commented out — not used in Old_nav pipeline, and incompatible with PointCloud2 driver output

```xml
<!-- Driver: PointCloud2 on /livox/lidar, no rviz -->
<include file="$(find livox_ros_driver2)/launch_ROS1/rviz_MID360.launch">
    <arg name="rviz_enable" value="false"/>
</include>

<!-- Localization: direct from driver, like Old_nav -->
<include file="$(find hdl_localization)/launch/hdl_localization.launch">
    <arg name="points_topic" value="/livox/lidar" />
</include>

<!-- Processor: DISABLED (expects CustomMsg, driver publishes PointCloud2) -->
<!-- <include file="$(find livox_cloudpoint_processor)/launch/lidar_filter_pointcloud.launch" /> -->
```

### Impact on Earlier Fixes
- **livox_cloudpoints_processor.cpp** Y/Z fix — still correct code, but node is now disabled
- **mixed_HAP_MID360_config.json** (roll=20.0) — now **unused** (`rviz_MID360.launch` loads `MID360_config.json` which already had roll=20.0)
- **imu_filter.cpp** sync — still active and necessary ✅
- **real_robot_transform.cpp** sync — still active and necessary ✅

### Current Pipeline (matches Old_nav)
```
Livox driver (MID360_config.json, xfer_format=0)
  → /livox/lidar (PointCloud2, frame: livox_frame)
  → hdl_localization (NDT matching against globalmap)
  → /odom, /aligned_points

imu_filter → /livox/imu (filtered) → hdl_localization (UKF prediction)
real_robot_transform: aft_mapped → gimbal_frame TF
```

---

## Cause 8 (2026-03-21): Frame ID mismatch + invert_imu_acc desync

### Symptom
```
[ERROR] canTransform: target_frame aft_mapped does not exist.
canTransform: source_frame livox_frame does not exist.
```
Localization still floats away even with PointCloud2 pipeline.

### Root Cause 1 — Frame ID Mismatch
The HIT `rviz_MID360.launch` defaults `msg_frame_id=livox_frame`, but Old_nav's defaults to `msg_frame_id=aft_mapped`. The entire rest of the pipeline expects `aft_mapped`:
- `hdl_localization`: `odom_child_frame_id=aft_mapped` → publishes TF `map → aft_mapped`
- `real_robot_transform`: `_3DLidar_frame=aft_mapped` → publishes TF `aft_mapped → gimbal_frame`

With `livox_frame`, the driver publishes pointcloud in a frame nobody creates TF for. hdl_localization can't match it against the `aft_mapped`-based TF tree.

### Root Cause 2 — invert_imu_acc parameter desync
After syncing `imu_filter.cpp` to Old_nav (which negates all 3 accel axes: `×= -gravity`), the `invert_imu_acc` parameter must also match:
- Old_nav: `imu_filter` (×-gravity) → hdl_localization (`invert_acc=true`, ×-1) → **net: +gravity**
- HIT was: `imu_filter` (×-gravity) → hdl_localization (`invert_acc=false`) → **net: -gravity**

The accel sign was inverted relative to Old_nav, causing localization drift.

### Fix Applied
Two overrides added to `HIT_intergration_test.launch`:
```xml
<include file="$(find livox_ros_driver2)/launch_ROS1/rviz_MID360.launch">
    <arg name="rviz_enable" value="false"/>
    <arg name="msg_frame_id" value="aft_mapped"/>  <!-- was livox_frame -->
</include>

<include file="$(find hdl_localization)/launch/hdl_localization.launch">
    <arg name="points_topic" value="/livox/lidar" />
    <arg name="invert_imu_acc" value="true" />      <!-- was false -->
</include>
```

### Full Parameter Sync Verification (Old_nav ↔ HIT)

| Parameter | Old_nav | HIT (now) | Match |
|-----------|---------|-----------|-------|
| msg_frame_id (driver) | aft_mapped | aft_mapped | ✅ |
| xfer_format (driver) | 0 (PointCloud2) | 0 (PointCloud2) | ✅ |
| rviz_enable (driver) | false | false | ✅ |
| MID360_config.json roll | 20.0° | 20.0° | ✅ |
| imu_filter.cpp code | ×-gravity, -20° rot | ×-gravity, -20° rot | ✅ (identical) |
| output_frame_id (imu) | "" (empty) | "" (empty) | ✅ |
| points_topic (hdl_loc) | /livox/lidar | /livox/lidar | ✅ |
| odom_child_frame_id | aft_mapped | aft_mapped | ✅ |
| invert_imu_acc | true | true | ✅ |
| invert_imu_gyro | false | false | ✅ |
| imu_topic | /livox/imu_filtered | /livox/imu_filtered | ✅ |
| robot_odom_frame_id | aft_mapped | aft_mapped | ✅ |
| NDT params (all) | identical | identical | ✅ |
| real_robot_transform.cpp | identical code | identical code | ✅ |
| _3DLidar_frame | aft_mapped | aft_mapped | ✅ |
| odom_frame | map | map | ✅ |

**Intentional differences** (different map):
| Parameter | Old_nav | HIT | Reason |
|-----------|---------|-----|--------|
| globalmap_pcd | Mar16.pcd | current.pcd | Different PCD map |
| init_pos_x | 6.798 | 0.0 | Origin differs per map |
| init_pos_y | -1.802 | 0.0 | Origin differs per map |

## Why the Old Fix Was Wrong
The previous fix edited `bot_sim`'s `threeD_lidar_merge_pointcloud.cpp` and `threeD_lidar_filter_pointcloud.cpp`. These are NOT included in `HIT_intergration_test.launch`. The HIT launch uses `livox_cloudpoint_processor` (from `ws_cloud`) instead. Those bot_sim edits had no effect on the running system.
