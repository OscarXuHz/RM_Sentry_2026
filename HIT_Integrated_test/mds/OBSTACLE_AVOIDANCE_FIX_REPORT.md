> **⚠️ HISTORICAL** — References the old dual-LiDAR robot (`_105` + `_3`).  Current system uses a single MID-360.  Fix principles were carried forward but specific code paths are superseded.

# Obstacle Avoidance Fix Report

**Date:** 2025-01  
**Scope:** Global planner (trajectory_generation) + Local planner (trajectory_tracking / MPC)

> **See also:** [PROBLEM_REPORT_CMD_VEL_FLUCTUATION.md](PROBLEM_REPORT_CMD_VEL_FLUCTUATION.md) for the cmd_vel oscillation root cause analysis.

---

## Symptoms Reported

1. **Global planner not avoiding isolated obstacles in the middle of the ground** — the A\* path goes straight through objects that the LiDAR clearly sees.
2. **MPC predicted route fluctuates violently when nearing real-time obstacles** — the local planned trajectory jitters between left/right avoidance on every MPC cycle.

---

## Root Cause Analysis

### Issue 1: Invisible Ground Obstacles

Three factors combined to make ground obstacles invisible:

| Factor | Code Location | Problem |
|--------|--------------|---------|
| **`l_data[]` memset reset** | `RM_GridMap.cpp` → `localPointCloudToObstacle()` | `memset(l_data, 0, GLXY_SIZE)` is called every LiDAR callback. Dynamic obstacles survive for exactly **one scan frame** (~100 ms). If the next scan doesn't see the obstacle (e.g., robot rotated), it vanishes from the map instantly. |
| **`search_height_min = 0.1`** | `global_searcher.launch` / `trajectory_planning.launch` | Points below Z = 0.1 m are discarded in `lidarCloudCallback` **before** any further processing. Obstacles shorter than 10 cm are completely invisible. |
| **`height_threshold = 0.15`** | Same launch files | In `localPointCloudToObstacle`, points with `Z < ground_height + 0.15` are treated as "ground" and skipped. Combined with the 10 cm Z cutoff, obstacles under ~15 cm are filtered twice. |

**Data flow where obstacles are lost:**

```
/aligned_points_local (LiDAR points)
    │
    ▼
lidarCloudCallback:  pt.z > 0.1 ?  ──NO──▶ DISCARDED     ← Fix 3a
    │ YES
    ▼
localPointCloudToObstacle:
    memset(l_data, 0, ...)           ← Fix 1 (persistence)
    height + 0.15 > pt.z ?  ──YES──▶ DISCARDED (ground)   ← Fix 3b
    │ NO
    ▼
    l_data[idx] = 1                  ← Fix 1 (persist counter)
    ...but reset to 0 next frame
```

### Issue 2: MPC Path Flickering Near Obstacles

Four factors combined to make the MPC trajectory unstable:

| Factor | Code Location | Problem |
|--------|--------------|---------|
| **Obstacle set rebuilt from scratch every cycle** | `localPointCloudToObstacle()` | `l_data[]` is fully zeroed then repopulated each scan. The obstacle constraint set input to the MPC changes dramatically frame-to-frame. |
| **Obstacle search window too small** | `local_planner.cpp` → `linearation()` | Only ±10 grid cells (= ±0.5 m at 0.05 m resolution) around each reference point are searched. Obstacles slightly outside this corridor appear/disappear as the reference shifts. |
| **Barrier penalty too sharp** | `RelaxedBarrierPenalty(mu=2.0, delta=0.1)` | `delta = 0.1` in **squared-distance** space means the penalty function transitions from zero to high over ≈ 0.03 m of actual distance. The solver is extremely sensitive to tiny obstacle position changes. |
| **Constraint set toggling** | `safe_constraint` ±3 window on `occ_flag[]` | The 7-point occupancy window can flip on/off as obstacles appear/disappear between frames, toggling the entire obstacle search for that horizon step. |

---

## Fixes Applied

### Fix 1: Obstacle Temporal Persistence (Code — Both RM_GridMap.cpp)

**Files modified:**
- `trajectory_generation/src/RM_GridMap.cpp`
- `trajectory_tracking/src/RM_GridMap.cpp`

**Before:** `memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t))` — full reset every scan.

**After:** Decay loop with persist counter:
```cpp
static const uint8_t OBS_PERSIST_FRAMES = 5;
for (int k = 0; k < GLXY_SIZE; ++k) {
    if (l_data[k] > 0) l_data[k]--;
}
```

And in `localSetObs()`:
```cpp
l_data[idx] = OBS_PERSIST;  // was: l_data[idx] = 1
```

Occupancy checks updated from `== 1` to `> 0`:
```cpp
// isOccupied: l_data[idx] == 1  →  l_data[idx] > 0
// isFree:     l_data[idx] < 1   →  l_data[idx] == 0
```

**Effect:** Obstacles persist for 5 scan frames (~0.5 s at 10 Hz). Objects behind the robot remain on the map long enough for the planner to route around them. The decay ensures truly-gone obstacles don't persist forever.

### Fix 2: Height Filter Parameters (Launch Files)

**Files modified:**
- `trajectory_generation/launch/global_searcher.launch`
- `trajectory_tracking/launch/trajectory_planning.launch`

| Parameter | Before | After | Effect |
|-----------|--------|-------|--------|
| `search_height_min` | 0.1 | -0.05 | Include LiDAR points at and slightly below ground level |
| `height_threshold` | 0.15 | 0.08 | Treat points 8 cm above BEV ground as obstacles (was 15 cm) |

**Why -0.05 for min height:**  
With the localization fix outputting Z ≈ 0.0, the ground plane is at Z = 0. LiDAR noise and slight calibration errors can put legitimate obstacle points at Z = -0.02 to 0.0. The `height_threshold` check in `localPointCloudToObstacle` still filters actual ground points, so this won't cause false positives.

**Why 0.08 for threshold:**  
Objects on the ground (e.g., supply boxes, low barriers) have LiDAR returns starting from Z ≈ 0.05–0.10 m. With the previous 0.15 threshold, anything shorter than ~15 cm was invisible. The new 8 cm threshold detects objects as low as ~8 cm while still filtering LiDAR noise from the flat ground.

### Fix 3: MPC Barrier Penalty Smoothing (Code Parameter)

**File modified:** `trajectory_tracking/src/local_planner.cpp`

```cpp
// Before:
RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(2.0, 0.1);
// After:
RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(2.0, 0.25);
```

**Effect:** The `delta` parameter controls how early the penalty ramp begins before the hard constraint boundary. Increasing from 0.1 to 0.25 (in squared-distance space) means:
- Penalty onset distance: √0.46 ≈ 0.68 m → √0.61 ≈ 0.78 m from obstacle center
- The gradient is spread over a wider region, so small changes in obstacle position produce smaller changes in the penalty, reducing solver sensitivity.

### Fix 4: MPC Obstacle Search Radius (Code)

**File modified:** `trajectory_tracking/src/local_planner.cpp`

```cpp
// Before: ±10 cells = ±0.5 m
for(int i = -10; i<=10; i++){
    for(int j = -10; j<=10; j++){

// After: ±15 cells = ±0.75 m
for(int i = -15; i<=15; i++){
    for(int j = -15; j<=15; j++){
```

**Effect:** The MPC now sees obstacles up to 0.75 m from each reference trajectory point (was 0.5 m). This means:
- Obstacles approaching from the side are detected earlier
- Fewer "surprise" appearances where an obstacle enters the search window between frames
- Combined with persistence (Fix 1), the constraint set changes smoothly

---

## Build Verification

```
catkin_make in HIT_code/sentry_planning_ws/
  ✓ tracking_node      — compiled, no errors
  ✓ trajectory_generation — compiled, no errors
  (pre-existing warnings only — no new warnings from fixes)
```

---

## Summary of All Changes

| # | Type | File(s) | Change |
|---|------|---------|--------|
| 1 | Code | `trajectory_generation/src/RM_GridMap.cpp` | `memset` → decay loop, `l_data=1` → `=5`, `==1` → `>0` |
| 2 | Code | `trajectory_tracking/src/RM_GridMap.cpp` | Same as above |
| 3a | Param | `global_searcher.launch` | `search_height_min` 0.1 → -0.05 |
| 3b | Param | `global_searcher.launch` | `height_threshold` 0.15 → 0.08 |
| 3c | Param | `trajectory_planning.launch` | `search_height_min` 0.1 → -0.05 |
| 3d | Param | `trajectory_planning.launch` | `height_threshold` 0.15 → 0.08 |
| 4 | Code | `trajectory_tracking/src/local_planner.cpp` | `delta` 0.1 → 0.25 in RelaxedBarrierPenalty |
| 5 | Code | `trajectory_tracking/src/local_planner.cpp` | MPC obstacle search ±10 → ±15 cells |

---

## Expected Behavior After Fix

1. **Ground obstacles:** Objects as low as ~8 cm above ground will be detected and persisted in both planners. A\* will route around them. If the robot turns away and the obstacle leaves the scan, it remains on the map for ~0.5 s.

2. **MPC stability near obstacles:** The wider search window, temporal persistence, and smoother barrier penalty combine to produce:
   - Gradually-changing constraint sets (no frame-to-frame flickering)
   - Smoother penalty gradients (solver doesn't jump between solutions)
   - Earlier obstacle detection (solver has time to plan smooth avoidance)

3. **Revert instructions:** If any fix causes issues:
   - Persistence: set `OBS_PERSIST_FRAMES = 1` (equivalent to original behavior)
   - Height params: restore `search_height_min=0.1`, `height_threshold=0.15`
   - Barrier delta: restore `delta=0.1`
   - Search radius: restore `±10`

---

## Session 3 Fixes (March 14, 2026)

### Additional Symptoms Reported

1. **Failing to avoid close/moving obstacles** — especially when drawing near, the MPC doesn't replan in time
2. **Overshooting the goal point** — robot passes the destination and oscillates trying to readjust
3. **Slow obstacle reaction time** — significant delay between obstacle detection and avoidance maneuver

### Fixes 17–25 Applied

| # | Fix | File(s) | Change |
|---|-----|---------|--------|
| 17 | Widen obstacle search | `local_planner.cpp` | `OBS_SEARCH_HALF` 15→25 (±1.25m), `MAX_OBS_PER_STEP` 5→8 |
| 18 | Dynamic obstacle clearance | `SentryRobotCollisionConstraint.h/.cpp` | Added `distance_threshold_dynamic_=0.25` (0.5m effective) for moving obstacles; static stays 0.09 |
| 19 | Stronger collision barrier | `local_planner.cpp` | Barrier µ 2.5→3.5 (Q/µ=43, safe ratio) |
| 20 | Zero velocity at goal | `local_planner.cpp` | Past-trajectory ref velocity → `Eigen::Vector3d::Zero()` instead of last segment speed |
| 21 | Deceleration ramp | `local_planner.cpp` | Linear v-scale over last 1.5m: `v *= max(0.05, dist/1.5)` |
| 22 | Arrival threshold | `tracking_manager.cpp` | `arrival_goal` at 0.3m (was 0.1m); removed premature replan at 1.0m |
| 23 | Faster replan debounce | `tracking_manager.cpp` | Buffer 25→15, collision threshold 15→10, tracking_low 6→4 |
| 24 | Reduced SQP iterations | `task.info` | `sqpIteration` 10→6 (warm-start converges in 3-4) |
| 25 | Aligned speed limits | `global_searcher.launch`, `trajectory_planning.launch` | `reference_desire_speed` 2.8→2.0, `local_v_max` 2.8→2.0 |

See [PLANNING_FIX_HISTORY.md](PLANNING_FIX_HISTORY.md) for full details on each fix.

---

## Session 4 Fixes (March 15, 2026)

### Symptoms Reported

1. **Ghost obstacles / violent detour paths** — After a moving obstacle passed through an area and left, the robot still treated those cells as blocked for ~2 seconds (OBS_PERSIST=20 in trajectory_generation). The topo-PRM could not route through the cleared corridor, producing wildly divergent paths that veered far off the normal topo map.
2. **Reference path diverges from predicted path** — The MPC predicted trajectory did not follow the reference trajectory. The solver appeared to "fight" the reference because position references advanced at full polynomial speed while velocity references were scaled down by the deceleration ramp (Fix 21). This created an internally inconsistent reference that no feasible MPC solution could satisfy.

### Root Causes

| Issue | Root Cause |
|-------|-----------|
| Ghost obstacles | Fix 15 set `OBS_PERSIST=20` (2 s at 10 Hz). A moving obstacle that crosses a corridor in <0.5 s leaves 2 s of ghost cells behind it. The global topo planner can't plan through those cells → violent alternative route. |
| Reference divergence | Fix 21 scaled `velocity_temp` inside `linearation()` but left `position_temp` unscaled. MPC state reference = `[x, y, speed, phi]` where positions come from the unscaled polynomial and speed comes from scaled velocity. The two are mutually inconsistent → solver compromise trajectory diverges from both. |

### Fixes 26–27 Applied

| # | Fix | File(s) | Change |
|---|-----|---------|--------|
| 26 | Reduce obstacle persistence in global planner | `trajectory_generation/src/RM_GridMap.cpp` | `OBS_PERSIST_FRAMES` 20→10, `OBS_PERSIST` 20→10 (2 s → 1 s ghost window) |
| 27 | Relocate deceleration ramp | `trajectory_tracking/src/local_planner.cpp` | (A) Removed velocity scaling from `linearation()` so position/velocity are consistent; (B) Added `goal_pos` variable in `getFightTrackingTraj()`; (C) Added deceleration ramp to `ref_speed` scalar at end of ref_speed loop: `ref_speed.back() *= max(0.05, dist_to_goal/1.5)` when within 1.5 m of goal |

### Why Fix 27 Works

The key insight is that MPC uses **four separate reference channels**:

| Channel | Source | Must be consistent with |
|---------|--------|------------------------|
| `ref_trajectory[i]` (x, y) | Polynomial evaluation | velocity (derivative) |
| `ref_velocity[i]` (vx, vy) | Polynomial derivative | position (integral) |
| `ref_speed[i]` (scalar) | `‖ref_velocity[i]‖` | — (independent in MPC cost) |
| `ref_phi[i]` (yaw) | `atan2(vy, vx)` | velocity direction |

Fix 21 broke the position↔velocity consistency by scaling velocity without scaling position. Fix 27 removes that inconsistency and instead applies the deceleration ramp only to the **scalar speed** channel (`ref_speed`), which enters the MPC cost independently as part of the state reference `[x, y, v, φ]`. The MPC can reduce speed near the goal without needing positions to "slow down" — positions simply mark where the robot should be, and the speed reference tells it how fast to be moving there.

See [PLANNING_FIX_HISTORY.md](PLANNING_FIX_HISTORY.md) for full details on each fix.
