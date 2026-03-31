# Localization & Map Alignment Fix Report

**Date:** 2026-03-20

> **See also:** [PROBLEM_REPORT_LOCALIZATION_TILT.md](PROBLEM_REPORT_LOCALIZATION_TILT.md) for root cause analysis of the persistent tilt issue. [UPSIDE_DOWN_POINTCLOUD_FIX_REPORT.md](UPSIDE_DOWN_POINTCLOUD_FIX_REPORT.md) for the full upside-down pointcloud fix (5 causes).

## Symptoms Reported

1. **~90° clockwise offset** between the published occupancy grid (`occfinal`) and `/globalmap` in RViz
2. **Localization divergence** after "2D Pose Estimate" in RViz — the `/3Dlidar` pointcloud appeared vertical and below `/globalmap`, then floated away
3. **Pointcloud upside-down** — after fixing 1 & 2, the live pointcloud still appeared inverted (floor up, ceiling down)

## Root Cause Analysis

### Cause 1 — Globalmap PCD from a different mapping run

| Property | occfinal (Point-LIO) | /globalmap (Mar16.pcd) |
|---|---|---|
| X range | [-10.9, 26.6] | [-14.1, 23.8] |
| Y range | [-30.6, 7.7] | [-7.7, 34.5] |
| Z range | [-0.5, 1.0] | [-2.0, 7.9] |
| Points | 68,559 | 804,264 |
| Floor normalised | Yes (floor ≈ Z=0) | No (Z median = 1.7) |
| X span / Y span | 37.4 / 38.3 | 37.9 / 42.2 |

The two maps cover the **same physical space** (~38 m across) but are in
**different coordinate frames** because they were built in separate mapping
sessions with different initial robot orientations.  Swapping X↔Y spans
(current Y 38.3 ≈ Mar16 X 37.9) confirms a **~90° rotation between frames**.

### Cause 2 — Init pose calibrated for Mar16.pcd

```
init_pos:  (6.798, -1.802, 0.0)
init_ori:  (w=0.7017, z=-0.7017)  →  yaw ≈ -90°
```

This was a valid starting pose **inside Mar16.pcd's coordinate frame**.
In the Point-LIO frame it placed the robot far from any map structure,
so NDT diverged immediately:

```
/odom position:    (88 526, 70 167, -0.15)
/odom orientation: roll=-97°, pitch=71°, yaw=-16°
/odom velocity:    (546, 1139, -2876)  ← UKF diverged
```

The wildly tilted orientation was broadcast as `map → aft_mapped` TF,
making the live LiDAR pointcloud appear **vertical** in RViz.

### Cause 3 — IMU accelerometer inversion for upside-down LiDAR

The old robot had the LiDAR **mounted upside-down**.  To compensate,
`hdl_localization.launch` set:

```xml
<arg name="invert_imu_acc" default="true" />
```

This negates all accelerometer readings, effectively flipping gravity in the
UKF's internal model.  When the LiDAR was upside-down, this made gravity
point in the correct direction.  But on this robot, the LiDAR is
**right-side-up**, so the inversion makes the UKF think gravity points
**upward** — the estimated pose's roll/pitch drifts to ~180°, and the
resulting `map → aft_mapped` TF renders the live pointcloud upside-down.

### Cause 4 — z_ceiling too low for localization map

After the z-band filter fix (z_ceiling = 0.4 in map builder), future maps
would contain only 0.9 m of vertical data.  NDT needs at least wall-height
(~1 m+) 3D structure to converge reliably.

---

## How "2D Pose Estimate" Works

| Step | Detail |
|---|---|
| RViz tool | User clicks position + drags arrow for heading on the map plane |
| Published message | `geometry_msgs/PoseWithCovarianceStamped` on `/initialpose` |
| Pose content | x, y, z=0 (ground plane), quaternion with only yaw (roll=pitch=0) |
| Subscriber | `HdlLocalizationNodelet::initialpose_callback()` |
| Action | Creates a **new** `PoseEstimator` (UKF) initialised at the given pose. Resets velocity, acc/gyro biases to zero, restarts IMU cool-down timer. |
| NDT then | From that pose, each incoming LiDAR scan is matched against `/globalmap` via NDT-OMP. If the initial pose is close to the real position (within ~2 m, ~30°), NDT converges and publishes `map → aft_mapped` TF. Otherwise it diverges. |
| Frame note | hdl_localization transforms the input scan from its native frame (gimbal_frame) → odom_child_frame_id (aft_mapped) via TF before running NDT. |

**Why it failed:** Mar16.pcd's coordinate system has no overlap with the
Point-LIO-built occfinal map, so clicking in RViz (which shows the
occfinal-aligned coordinate frame) placed the NDT seed at a location that
doesn't exist in the globalmap.  NDT immediately diverged.

---

## Fixes Applied

### Fix 1 — Replace globalmap PCD

**File:** `hdl_localization/launch/hdl_localization.launch`

```diff
- <param name="globalmap_pcd" value="$(find hdl_graph_slam)/map/Mar16.pcd" />
+ <param name="globalmap_pcd" value="/home/sentry/AstarTraining/RM_Sentry_2026/HIT_Integrated_test/pcd/current.pcd" />
```

This makes the 3D reference map (`/globalmap`) and the 2D occupancy grid
(`occfinal`) derive from the **same Point-LIO mapping session**, eliminating
the 90° frame offset.

### Fix 2 — Update initial pose for Point-LIO origin

**File:** `hdl_localization/launch/hdl_localization.launch`

```diff
- <param name="init_pos_x" value="6.798271894454956" />
- <param name="init_pos_y" value="-1.80238938331604" />
- <param name="init_pos_z" value="0.0" />
- <param name="init_ori_w" value="0.7017" />
- <param name="init_ori_x" value="0.0" />
- <param name="init_ori_y" value="0.0" />
- <param name="init_ori_z" value="-0.7017" />
+ <param name="init_pos_x" value="0.0" />
+ <param name="init_pos_y" value="0.0" />
+ <param name="init_pos_z" value="0.0" />
+ <param name="init_ori_w" value="1.0" />
+ <param name="init_ori_x" value="0.0" />
+ <param name="init_ori_y" value="0.0" />
+ <param name="init_ori_z" value="0.0" />
```

Point-LIO maps start at the robot's physical starting location = coordinate
origin (0, 0, 0) facing forward (yaw = 0).  The identity quaternion (w=1)
means "facing the Point-LIO X-axis direction."

### Fix 3 — Disable IMU accelerometer inversion

**File:** `hdl_localization/launch/hdl_localization.launch`

```diff
- <arg name="invert_imu_acc" default="true" />
+ <arg name="invert_imu_acc" default="false" />
```

The old robot had the LiDAR mounted upside-down, so accelerometer readings
needed negation for the gravity vector to point correctly.  This robot's
LiDAR is right-side-up — the raw accelerometer already gives the correct
gravity direction.  With `invert_imu_acc=true`, the UKF's gravity estimate
was flipped, causing the pose (and therefore the entire pointcloud) to
appear upside-down in RViz.

### Fix 4 — Restore map builder z_ceiling to 3.0

**File:** `HIT_pointlio_map_builder.launch`

```diff
- <arg name="z_ceiling" default="0.4" ... />
+ <arg name="z_ceiling" default="3.0" ... />
```

The localization PCD needs full wall height for NDT feature matching.
The 2D occupancy grid generation (`pcd_to_maps.py`) already applies its
own `_Z_CEILING = 0.4` independently, so planning maps remain thin.

---

## Architecture Summary

```
Point-LIO map builder
    │
    ▼
current.pcd  (floor-normalised, Z ∈ [-0.5, 3.0])
    │
    ├──▶ hdl_localization  (globalmap for NDT scan matching)
    │       └── /globalmap (PointCloud2, frame: "map")
    │       └── /odom, TF: map → aft_mapped
    │
    └──▶ pcd_to_maps.py  (Z-band filter → 2D occupancy grid)
            └── occfinal.png, bevfinal.png, occtopo.png
            └── Published as OccupancyGrid in RViz (frame: "map")
```

Both the 3D reference map and the 2D planning maps now come from the
**same PCD in the same coordinate frame**, so they are aligned in RViz.

---

## TF Frame Chain

```
map  (global reference, published by hdl_localization)
 └── aft_mapped  (LiDAR-estimated pose, odom_child_frame_id)
      └── gimbal_frame  (robot chassis, from real_robot_transform)
```

- `map → aft_mapped`: NDT localization result (hdl_localization, ~5 Hz)
- `aft_mapped → gimbal_frame`: static offset (real_robot_transform, 20 Hz)
- Soft Z constraint: `z ∈ [init_pos_z ± 0.15]` prevents vertical drift

---

## Next Steps After Restarting

1. **Re-build map** (recommended for better localization):
   ```bash
   roslaunch .../HIT_pointlio_map_builder.launch
   # Walk the robot around, then Ctrl-C
   cd .../pcd/ && mv scans.pcd current.pcd
   python3 .../pcd_to_maps.py --pcd current.pcd
   ```
   The new map will have z_ceiling=3.0 → full walls for NDT.

2. **Launch integration test:**
   ```bash
   roslaunch .../HIT_intergration_test.launch
   ```
   Verify in RViz that `/globalmap` and `occ_image_publisher/grid` align.

3. **Use "2D Pose Estimate"** if the robot starts at a different position:
   - Click the approximate location on the map, drag arrow for heading
   - The live pointcloud (`/aligned_points`) should snap into alignment

4. **Tune init pose** once you find a repeatable starting spot:
   - Echo `/odom` while standing at the start position
   - Copy the x, y, yaw values into `init_pos_x/y` and `init_ori_w/z`

---

## Files Changed

| File | Change |
|---|---|
| `sim_nav/src/hdl_localization/launch/hdl_localization.launch` | globalmap → current.pcd, init pose → origin, invert_imu_acc → false |
| `HIT_pointlio_map_builder.launch` | z_ceiling 0.4 → 3.0 |
