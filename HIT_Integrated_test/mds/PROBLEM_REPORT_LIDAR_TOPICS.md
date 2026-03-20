# Problem Report: LiDAR Topic Mismatch

## Problem Statement
The localization process and mapping process use different LiDAR topics, and the
point-cloud filter node has a critical bug that drops one of the two LiDARs
entirely.

## System Overview — LiDAR Topic Dataflow

| Stage | Node | Input Topic(s) | Output Topic(s) |
|-------|------|----------------|-----------------|
| Raw | Livox driver | — | `/livox/lidar_192_168_1_105` (left), `/livox/lidar_192_168_1_3` (right) |
| Merge | `threeD_lidar_merge_pointcloud` | `_105` + `_3` | `/3Dlidar` (combined PointCloud2) |
| Filter | `threeD_lidar_filter_pointcloud` | `_105` + `_3` | `/filted_topic_3d` (3D filtered), `/grid` (2D dilated) |
| Localization | `hdl_localization` | `/filted_topic_3d` | `/odom`, `/aligned_points` |
| Mapping | Point-LIO | `/livox/lidar_192_168_1_3` | PCD map file |

## Root Cause

### Bug 1: Filter subscribes to the same LiDAR twice

**File:** `Navigation-filter-test/ws_cloud/src/livox_cloudpoint_processor/launch/lidar_filter_pointcloud.launch`

```xml
<!-- BEFORE (BUG): both inputs are _105 — right LiDAR (_3) is ignored -->
<param name="scan_topic_left" type="string" value="/livox/lidar_192_168_1_105" />
<param name="scan_topic_right" type="string" value="/livox/lidar_192_168_1_105" />
```

The filter node (`livox_cloudpoint_processor`) is designed to accept two raw
LiDAR topics and combine them after filtering.  Both `scan_topic_left` **and**
`scan_topic_right` were set to the **same** topic (`_105`), making the right
LiDAR (`_3`) completely invisible to the filter.

**Consequence:** The downstream topics `/filted_topic_3d` and `/grid` contained
only left-LiDAR data, halving the field-of-view.  `hdl_localization` matched
half-coverage scans against a map built from the right LiDAR — a significant
FoV mismatch.

### Issue 2: Mapping uses a single LiDAR (by design)

Point-LIO subscribes to `/livox/lidar_192_168_1_3` (right LiDAR) with its
tightly-coupled IMU (`/livox/imu_192_168_1_3`).  Changing this to the combined
`/3Dlidar` topic is **not recommended** because:

1. Point-LIO performs per-point motion compensation using the LiDAR-IMU
   extrinsics calibrated for `_3`.  The combined cloud includes `_105` points
   whose extrinsics differ — applying `_3`'s extrinsics would introduce
   geometric errors.
2. The merge node applies a hardcoded rigid-body transform
   (`threeD_lidar_merge_pointcloud.cpp`) that converts both clouds to a common
   frame.  These transforms do not match Point-LIO's expected raw-sensor frame.

Single-LiDAR mapping is acceptable because NDT-based localization
(`hdl_localization`) is robust to moderate FoV differences between the live scan
and the map — as long as there is sufficient overlap.

## Fix Applied

```xml
<!-- AFTER: right LiDAR correctly set to _3 -->
<param name="scan_topic_left" type="string" value="/livox/lidar_192_168_1_105" />
<param name="scan_topic_right" type="string" value="/livox/lidar_192_168_1_3" />
```

**File changed:**
`Navigation-filter-test/ws_cloud/src/livox_cloudpoint_processor/launch/lidar_filter_pointcloud.launch`

No rebuild needed — this is a launch-file parameter change.

## Post-Fix Topic Flow

```
/livox/lidar_192_168_1_105 ─┐
                             ├─► threeD_lidar_filter_pointcloud ─► /filted_topic_3d ─► hdl_localization
/livox/lidar_192_168_1_3  ──┘                                  └─► /grid ──────────► HIT planners

/livox/lidar_192_168_1_3  ────► Point-LIO (mapping only)

/livox/lidar_192_168_1_105 ─┐
                             ├─► threeD_lidar_merge_pointcloud ─► /3Dlidar (available but not primary input)
/livox/lidar_192_168_1_3  ──┘
```

## Verification

After launching the integration stack:
```bash
rostopic hz /filted_topic_3d   # should show ~50 Hz with combined data
rostopic echo /filted_topic_3d --noarr | head -5  # check frame_id
```

If the robot's obstacle avoidance visibly improves (fewer phantom obstacles,
wider FoV coverage), the fix is confirmed.
