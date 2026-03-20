# Z-Origin & Floor Detection Report

**Date:** 2026-03-20  
**Status:** Fixed & rebuilt (3 iterations)

---

## How Point-LIO Sets the Z=0 Plane

Point-LIO's world frame ("camera_init") is established at the very first valid
LiDAR frame by a one-shot initialisation in `laserMapping.cpp`:

### Step 1 — Zero-initialised state
The Kalman filter states (`state_input`, `state_output`) start at:
- `pos = [0, 0, 0]` (origin at the IMU position)
- `rot = Identity` (no rotation)
- `gravity = [0, 0, 0]`

### Step 2 — Gravity measurement
If the robot starts stationary (`non_station_start = false`), Point-LIO
accumulates ~100 IMU samples and takes their mean acceleration as the measured
gravity vector **in the IMU body frame**:
```
state_in.gravity = -mean_acc * G / |acc|
```
If pre-calibrated (`non_station_start = true`), the value from
`mapping/gravity_init` in avia.yaml is used directly ([0, 0, −9.81]).

### Step 3 — Gravity alignment (`gravity_align = true`)
`ImuProcess::Set_init()` computes a rotation matrix `rot_init` that maps the
measured gravity direction onto the target world-frame gravity `[0, 0, −9.81]`
using a Rodrigues axis-angle formula:
```
axis  = gravity_world × gravity_measured / |cross product|
angle = acos( gravity_world · gravity_measured / |both| )
rot_init = Exp(axis * angle)
```
This rotation is applied to `state.rot`, making the **Z-axis point UP**
(perpendicular to gravity). The position remains `[0, 0, 0]`, so **z=0 is
at the IMU/LiDAR starting height**, not at the floor.

### Step 4 — lidar_height offset
After gravity alignment and before pushing the state into the Kalman filter,
the `mapping/lidar_height` parameter offsets `pos.z()`:
```cpp
if (lidar_height > 0.0) {
    state_in.pos(2)  = lidar_height;   // e.g. 0.3
    state_out.pos(2) = lidar_height;
}
kf_input.change_x(state_in);
kf_output.change_x(state_out);
```
This shifts the world-frame origin **down** by `lidar_height` metres, placing
z=0 at the floor plane (~0.3 m below the LiDAR). All subsequent points
transformed via `pointBodyToWorld()` inherit this offset.

### Step 5 — Point transformation chain (runtime)
Every LiDAR point goes through:
```
P_world = state.rot * (R_LI * P_lidar + T_LI) + state.pos
                       ^^^^^^^^^^^^^^^^^^^^        ^^^^^^^^
                       extrinsic calibration       includes lidar_height
```
`R_LI` and `T_LI` are from `mapping/extrinsic_R` / `mapping/extrinsic_T` in
avia.yaml (LiDAR-to-IMU offset: 4cm forward, 2cm right, −2.8cm down).

### Step 6 — Post-processing: `normalize_floor_in_saved_map()` (on PCD save)
When Point-LIO shuts down and saves the PCD, if `pcd_save/level_floor = true`:

1. **Floor detection** — builds a Z-histogram (5 cm bins), smooths it with a
   ±5-bin (~50 cm) moving average, then scans **upward from the bottom** to
   find the **first bin exceeding 30% of the global peak** — the lowest dense
   horizontal surface. This is the floor, not the ceiling.
2. **Refinement** — finds the local maximum around that starting bin (±10 bins)
   to snap to the exact floor peak.
3. **Inlier selection** — collects points within ±50 cm of the floor estimate
   (first pass) or ±10 cm (refinement passes) as floor inliers.
4. **PCA floor fitting** — computes the covariance matrix of floor inliers and
   extracts the smallest eigenvector → floor normal.
5. **Rotation** — rotates the **entire cloud around the origin** so the floor
   normal aligns with +Z (levels any residual tilt).
6. **Iterates** up to `pca_max_iterations` times, converging when residual
   tilt < 0.01°.
7. **Z-shift** — recomputes the floor Z using the same lowest-peak method
   after rotation, takes the median of nearby points, and shifts all
   points so `floor_median = floor_target_z` (default 0.0).

### Step 7 — Z-band filter (on PCD save, after floor normalisation)
After floor normalisation, the z-band filter removes points outside
`[pcd_z_floor, pcd_z_ceiling]`, discarding residual sub-floor noise and
above-ceiling returns.

### Complete coordinate chain
```
LiDAR pt  →  extrinsic (R_LI, T_LI)  →  IMU body frame
          →  state.rot (gravity-aligned) + state.pos (includes lidar_height)
          →  world frame ("camera_init", z=0 at floor)
          →  [on save] PCA floor leveling  →  Z-shift to target_z
          →  [on save] z_band filter [z_floor, z_ceiling]
          →  saved PCD
```

---

## Bug History

### Bug 1 — Ceiling-only z-filter (first fix)
The initial z-filter only had an upper bound (`z <= 3.0`) with no lower bound.
This removed nothing because no points exceeded 3m.

### Bug 2 — z=0 at LiDAR, not floor (second fix)
Point-LIO initialised `pos=[0,0,0]` at the IMU. With the LiDAR ~0.3m above
the floor, the floor appeared at Z≈−0.3 and everything below was legitimate
data. Added `mapping/lidar_height` to offset the origin.

### Bug 3 — Floor detection picked the ceiling (this fix)

**Environment:** The lab has two layers — a floor where the robot operates,
and a ceiling ~3m above with an open space for tubes and vents above it.
The LiDAR sees both surfaces.

**Problem:** `normalize_floor_in_saved_map()` used the **global histogram
mode** (most populated Z-bin) to identify the floor. In a two-layer lab:

| Surface | Z (raw, pre-norm) | Histogram density |
|---------|-------------------|-------------------|
| Actual floor | Z ≈ −0.3 (below LiDAR) | Dense (flat surface, but seen at grazing angles) |
| Wall zone near LiDAR | Z ≈ 0 | **Very dense** (closest to sensor) |
| Ceiling + above | Z ≈ +2.5 | Dense (flat surface, many returns from below) |

The **global mode** landed on the wall zone / ceiling area (Z ≈ 0 in raw
coords) because vertical surfaces near the LiDAR had the most returns per
5cm bin. The algorithm then:
1. Treated this as "the floor"
2. Fitted a PCA plane through the ceiling/wall zone
3. Shifted Z so THIS surface → Z=0
4. The **actual floor** ended up at Z ≈ −3.2

This meant the z-band filter `[-0.5, 3.0]` removed ~68% of points — all the
legitimate floor and lower-wall returns.

**Evidence from scans.pcd (histogram, 0.25m bins):**
```
Z=[-3.25, -3.00):  23,673  ← spike = actual floor (now misplaced)
Z=[-3.00, -2.75):  18,032  ← walls
...uniform ~20k per bin...
Z=[-0.25,  0.00):  35,049  ← wrongly detected "floor" (really wall/ceiling zone)
Z=[ 0.00, +0.25):  32,980
Z=[ 0.25, +0.50):  11,960
Z=[ 2.00, +2.25):   3,202  ← tubes/vents above ceiling
```

**Fix — Lowest-peak floor detection:**

Replaced the global-mode search with a "scan upward from bottom" strategy:

```cpp
// 1. Build Z histogram (5cm bins)
// 2. Smooth with ±5-bin (~50cm) moving average
// 3. Find global peak height in smoothed histogram
// 4. Scan UPWARD from bin 0: first bin exceeding 30% of peak = floor
// 5. Refine: snap to local max within ±10 bins
```

This correctly identifies the floor (the **lowest** dense horizontal surface)
even when the ceiling or wall zone has more total returns. The 30% threshold
ensures minor noise bins at the bottom of the Z range are skipped.

Applied to **both** floor-detection locations:
- PCA loop (for plane fitting and leveling)
- Z-shift section (for final floor median alignment)

---

## All Changes (Cumulative)

| File | Change |
|------|--------|
| `Point-LIO/src/parameters.h` | Added `extern double lidar_height`, `extern double pcd_z_floor` |
| `Point-LIO/src/parameters.cpp` | Added `lidar_height` + `pcd_z_floor` variables and param loads |
| `Point-LIO/src/laserMapping.cpp` | `lidar_height` z-origin shift at init; **lowest-peak** floor detection (replaced global mode) in both PCA loop and Z-shift; z-band filter `[z_floor, z_ceiling]`; `#include <limits>` |
| `HIT_pointlio_map_builder.launch` | Added `lidar_height` arg (0.3), `z_floor` arg (−0.5), `z_ceiling` arg (3.0) + params |
| `pcd_to_maps.py` | Z-band filter `[−0.5, 3.0]` |

## Build

```
catkin_make --pkg point_lio  →  [100%] Built target pointlio_mapping
```

## Usage

```bash
# Default: lidar_height=0.3, z_floor=-0.5, z_ceiling=3.0
roslaunch .../HIT_pointlio_map_builder.launch

# Custom LiDAR height / z-band:
roslaunch .../HIT_pointlio_map_builder.launch lidar_height:=0.4 z_floor:=-0.3 z_ceiling:=2.5

# Disable z-origin shift (z=0 at LiDAR, like stock Point-LIO):
roslaunch .../HIT_pointlio_map_builder.launch lidar_height:=0.0
```
