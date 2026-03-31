# Map Gradient Tilt Fix — Detailed Report

## Problem Statement

The Point-LIO SLAM-generated PCD map exhibited a gradient (tilt) — the Z-coordinate
of the floor systematically varied with position, creating a slope in the direction
roughly towards `(1, 1, -0.0..)`. This was caused by a combination of overcorrected
IMU gravity initialization and inadequate post-processing floor normalization.

---

## Root Causes Identified

### 1. Overcorrected `gravity_init` from old arena calibration

The `gravity_init` parameter in `avia.yaml` was set to arena-specific values from a
previous venue:

```yaml
# BAD — overcorrected for old arena, causes tilt in new arena
gravity_init: [-0.141128, -0.385181, -9.806548]
```

This vector biases Point-LIO's initial gravity estimate, causing the SLAM coordinate
frame to be rotated relative to true gravity. When the robot moved to a new arena,
the calibration became invalid, introducing a systematic tilt.

**Files affected:**
- `RM_Sentry_2026/HIT_Integrated_test/sim_nav/src/Point-LIO/config/avia.yaml`
- `HIT_Integrated_test/sim_nav/src/Point-LIO/config/avia.yaml`
- `sim_nav/src/Point-LIO/config/avia.yaml`

### 2. Floor normalization algorithm failures

The original 2-pass PCA floor normalization had multiple flaws:

| Issue | Effect |
|-------|--------|
| Bottom-8% Z-quantile floor selection | Included extreme outliers (Z = -50.08) |
| No outlier rejection (IQR/MAD/RANSAC) | PCA corrupted by non-floor points |
| Rotation around floor-subset centroid | Shifted non-floor points unpredictably |
| Bottom-40% histogram mode search | Missed floor when Z-origin wasn't at floor level |
| Fixed ±10cm inlier band on refined passes | Too tight for tilted floors (17cm Z variation over 20m at 0.5°) |

**Normalization log from failed attempt:**
```
Pass 0: 10.79° tilt, 16781 pts (quantile — outlier contaminated!)
Pass 1:  0.57° tilt,  7582 pts (refined band)
Pass 2:  0.62° tilt,  7967 pts (DIVERGING)
Pass 3:  0.59° tilt,  8482 pts
Pass 4:  0.53° tilt,  9050 pts (still 0.53° — failed to converge)
```

### 3. Missing `gravity` YAML key (regression — segfault)

During the initial fix, the `gravity: [0.0, 0.0, -9.810]` line in `avia.yaml` was
accidentally removed along with a commented block. Since YAML duplicate keys silently
override, this left `mapping/gravity` as an empty vector. The `VEC_FROM_ARRAY(gravity)`
macro then dereferenced index 0 of an empty `std::vector`, causing a null-pointer
segfault (exit code -11).

---

## Fixes Applied

### Fix 1: Revert `gravity_init` to default

**Files:** All three `avia.yaml` copies

Commented out the overcorrected arena-specific `gravity_init` and restored the
neutral default:

```yaml
gravity_init: [0.0, 0.0, -9.810]
# OLD ARENA — DO NOT USE:
# gravity_init: [-0.141128, -0.385181, -9.806548]
```

### Fix 2: Gravity vector safety check

**File:** `parameters.cpp`

Added bounds checking for `gravity` and `gravity_init` vectors loaded from YAML.
If either has fewer than 3 elements, defaults to `[0, 0, -9.81]` with a warning:

```cpp
if (gravity.size() < 3) {
    ROS_WARN("[params] gravity has < 3 elements, defaulting to [0,0,-9.81]");
    gravity = {0.0, 0.0, -9.81};
}
```

### Fix 3: Restore `gravity` YAML key

**File:** `avia.yaml`

Restored the `gravity: [0.0, 0.0, -9.810]` line that was accidentally deleted.

### Fix 4: Rewrite floor normalization algorithm (C++)

**File:** `laserMapping.cpp` — `normalize_floor_in_saved_map()`

Complete rewrite with the following improvements:

| Feature | Old | New |
|---------|-----|-----|
| Floor detection | Bottom-8% Z quantile | **Global histogram mode** (densest Z bin) |
| Histogram search | Bottom 40% of Z range | **Full Z range** (floor can be at any Z) |
| Bin width | 10 cm | **5 cm** (better resolution) |
| Initial candidate band | ±20 cm | **±50 cm** (captures full tilted floor) |
| Outlier rejection | None | Histogram mode rejects sparse Z regions |
| Rotation center | Floor-subset centroid | **Origin** (preserves coordinate system) |
| Convergence | 0.005° threshold | 0.01° threshold |
| Max iterations | 2 (hardcoded) | **Configurable** via `pcd_pca_max_iterations` (default 5) |

### Fix 5: New configurable parameter

**Files:** `parameters.h`, `parameters.cpp`, `HIT_pointlio_map_builder.launch`

Added `pcd_pca_max_iterations` parameter (default: 5), loaded from
`pcd_save/pca_max_iterations` in the launch file.

### Fix 6: Offline PCD re-leveling script (RANSAC)

**File:** `HIT_Integrated_test/pcd/relevel_pcd.py`

Python script for immediate offline re-leveling of existing PCD files using
**RANSAC plane fitting** — far more robust than iterative PCA:

1. **Global histogram mode** finds floor Z (densest Z bin across full range)
2. Selects candidates within ±50cm of floor Z
3. **RANSAC** (3000 iterations, ±3cm inlier threshold) fits the floor plane
4. **PCA refinement** on all points within ±5cm of RANSAC plane
5. Single rotation aligns floor normal with Z-axis
6. Z-shift places floor at Z = 0

---

## Results

### Before (original map):
```
Floor tilt:  2.95°
Gradient:    0.0516
Floor plane: z = -0.022x + 0.047y - 0.534
Z range:     [-50.08, 7.03]
Quadrant x>mean,y<mean: 36 points, Z_mean = -8.07  (severe outlier contamination)
```

### After (RANSAC re-leveled):
```
Floor tilt:  0.079°     (37x improvement)
Gradient:    0.0014      (37x improvement)
Floor plane: z = -0.0011x + 0.0008y + 0.008
Z range:     [-4.14, 3.86]
Quadrant uniformity:
  x>m,y>m: Z_mean = 0.0108
  x>m,y<m: Z_mean = 0.0085
  x<m,y>m: Z_mean = 0.0211
  x<m,y<m: Z_mean = 0.0159
```

**Tilt reduced from 2.95° to 0.079° (97.3% reduction)**

---

## Files Modified Summary

| File | Change |
|------|--------|
| `HIT/.../Point-LIO/config/avia.yaml` | Reverted gravity_init, restored gravity key |
| `RM_Sentry_2026/.../Point-LIO/config/avia.yaml` | Reverted gravity_init |
| `sim_nav/src/Point-LIO/config/avia.yaml` | Reverted gravity_init, restored gravity key |
| `Point-LIO/src/laserMapping.cpp` | Rewrote normalize_floor_in_saved_map() |
| `Point-LIO/src/parameters.h` | Added pcd_pca_max_iterations extern |
| `Point-LIO/src/parameters.cpp` | Added param loading + gravity safety checks |
| `HIT_pointlio_map_builder.launch` | Added pca_max_iterations arg |
| `HIT_Integrated_test/pcd/relevel_pcd.py` | New RANSAC re-leveling script |

## Files in PCD directory

| File | Description |
|------|-------------|
| `scans.pcd` | Re-leveled map (0.079° residual tilt) |
| `scans_original.pcd` | Backup of pre-leveling map |
| `relevel_pcd.py` | Offline re-leveling script (reusable) |

---

## Usage

### Re-level an existing PCD file:
```bash
cd HIT_Integrated_test/pcd
python3 relevel_pcd.py                          # auto-detects scans.pcd
python3 relevel_pcd.py input.pcd output.pcd     # explicit paths
```

### Build a new map (auto-leveling on save):
```bash
roslaunch HIT_pointlio_map_builder.launch
# Ctrl+C to save PCD — normalization runs automatically
```
