# Problem Report: Localization Tilt towards (1, -1, 0)

> **Status:** ✅ Fix deployed (Fix 32 + Fix 33c in PLANNING_FIX_HISTORY.md)
> Fix 32: IMU alignment (imu_filter_3 + switch to _3_filtered)
> Fix 33c: init_pos_z=0.3 + soft z-constraint (±0.15m)

## Problem Statement
The localization output (`/odom` from `hdl_localization`) exhibits a persistent
upward tilt towards the (1, -1, 0) direction, even after the PCD floor
normalization fix (P1/P2) was applied to the saved map.

## Root Cause

### IMU Mismatch Between Mapping and Localization

| Stage | IMU Used | Source |
|-------|----------|--------|
| **Mapping** (Point-LIO) | `/livox/imu_192_168_1_3` (right LiDAR, raw) | `avia.yaml` |
| **Localization** (hdl_localization) | `/livox/imu_192_168_1_105_filtered` (left LiDAR, filtered) | `hdl_localization.launch` |

`hdl_localization` uses IMU data for **inter-scan pose prediction**.  Between
pointcloud arrivals, it integrates accelerometer and gyroscope readings to
predict where the robot has moved.  If the IMU's gravity vector reference
differs from the one used during mapping, each prediction step introduces a
small systematic tilt error.

Even though both IMUs (`_3` and `_105`) are on the same chassis, they may have:
- Slightly different mounting orientations (even 0.5° produces significant drift
  over time)
- Different bias calibrations
- Different noise characteristics

The gravity vector reported by `_105`'s IMU would differ from `_3`'s, meaning
the NDT scan-matching receives a tilted initial guess at every iteration.  While
NDT corrects this partially, the residual tilt accumulates — manifesting as the
observed (1, -1, 0) tilt direction.

### Contributing Factor: FoV Mismatch (Fixed Separately)

The filter bug (both inputs set to `_105`) meant localization received
half-coverage scans while the map was built from `_3`.  The NDT matching had less
geometric constraint and could settle into a slightly tilted local minimum.
This is addressed in the separate LiDAR topics fix.

## Fix Applied

### 1. Added IMU filter for `_3` (the mapping IMU)

**File:** `HIT_Integrated_test/sim_nav/src/bot_sim/launch_real/imu_filter.launch`

```xml
<!-- BEFORE: only _105 IMU was filtered -->
<node pkg="bot_sim" type="imu_filter" name="imu_filter">
    <param name="input_imu_topic" value="/livox/imu_192_168_1_105" />
    <param name="output_imu_topic" value="/livox/imu_192_168_1_105_filtered" />
</node>

<!-- AFTER: both IMUs filtered; _3 added for localization use -->
<node pkg="bot_sim" type="imu_filter" name="imu_filter_105">
    <param name="input_imu_topic" value="/livox/imu_192_168_1_105" />
    <param name="output_imu_topic" value="/livox/imu_192_168_1_105_filtered" />
</node>
<node pkg="bot_sim" type="imu_filter" name="imu_filter_3">
    <param name="input_imu_topic" value="/livox/imu_192_168_1_3" />
    <param name="output_imu_topic" value="/livox/imu_192_168_1_3_filtered" />
</node>
```

### 2. Changed `hdl_localization` to use `_3`'s filtered IMU

**File:** `HIT_Integrated_test/sim_nav/src/hdl_localization/launch/hdl_localization.launch`

```xml
<!-- BEFORE -->
<arg name="imu_topic" default="/livox/imu_192_168_1_105_filtered" />

<!-- AFTER: matches Point-LIO mapping IMU source -->
<arg name="imu_topic" default="/livox/imu_192_168_1_3_filtered" />
```

## Rationale

By aligning the localization IMU with the mapping IMU:

1. The gravity vector reference is identical → inter-scan prediction doesn't
   introduce systematic tilt
2. Any remaining IMU biases are consistent between map-building and live
   operation → NDT converges to the correct (flat) orientation
3. The floor normalization in the PCD is preserved because the same IMU gravity
   reference is used to interpret both the map and the live scans

## Verification

After launching:
```bash
# Monitor the localization pose — Z-component of orientation quaternion should
# be stable and close to the init_ori values
rostopic echo /odom/pose/pose/orientation

# Check pitch/roll angles — should be near zero
rostopic echo /odom | python3 -c "
import sys, yaml, math
for doc in yaml.safe_load_all(sys.stdin):
    if doc is None: continue
    q = doc['pose']['pose']['orientation']
    roll = math.atan2(2*(q['w']*q['x']+q['y']*q['z']), 1-2*(q['x']**2+q['y']**2))
    pitch = math.asin(max(-1, min(1, 2*(q['w']*q['y']-q['z']*q['x']))))
    print(f'roll={math.degrees(roll):.2f}° pitch={math.degrees(pitch):.2f}°')
"
```

Expected: roll and pitch should remain within ±1° during normal operation
(previously drifted by several degrees in the (1,-1,0) direction).

## If Tilt Persists

If residual tilt remains after these fixes, potential next steps:
1. **NDT convergence tuning**: increase `ndt_resolution` or switch
   `ndt_neighbor_search_method` from `DIRECT7` to `DIRECT1` for tighter matching
2. **Gravity alignment in hdl_localization**: the nodelet can normalize the
   IMU-predicted gravity to match the PCD's floor plane
3. **Explicit floor constraint**: add a floor-plane constraint to the NDT cost
   function that penalises Z-tilt
