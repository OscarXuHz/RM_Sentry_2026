> **⚠️ HISTORICAL** — Root cause was the old dual-LiDAR single-filter bug, which cannot occur on the current single MID-360 hardware.

# Problem Report: cmd_vel Fluctuation and Excessive Global Replanning

> **See also:** [OBSTACLE_AVOIDANCE_FIX_REPORT.md](OBSTACLE_AVOIDANCE_FIX_REPORT.md) for the planner parameter fixes that resolved this.

## Problem Statement
The `/cmd_vel` topic oscillates violently between positive and negative values,
and the global trajectory is frequently re-planned even when the path appears
clearly navigable.

## Root Causes

### Cause 1: Single-LiDAR Filter Bug (Primary)

**Critical bug in `lidar_filter_pointcloud.launch`:** Both `scan_topic_left` and
`scan_topic_right` were set to `/livox/lidar_192_168_1_105`, meaning the right
LiDAR (`_3`) was completely ignored by the obstacle filter.

**How this causes cmd_vel oscillation:**

```
[Missing _3 data] → Obstacles on right side undetected
  → Global planner routes through actual obstacles
  → Robot approaches obstacle  → obstacle enters _105's FoV
  → Suddenly appears in obstacle map → tracking error > 0.4m
  → Replan triggered → new path → cycle repeats
```

```
[Missing _3 data] → Half-FoV obstacle map has gaps
  → Local MPC sees incomplete obstacle data → unstable cost function
  → MPC output oscillates between "go forward" and "go around/backward"
  → cmd_vel sign-flips
```

### Cause 2: DISPENSE Mode Velocity Reversal

In `tracking_manager.cpp`, when the robot is "stuck" (speed < 0.8 m/s for 1.2s
with average < 0.3 m/s), it enters DISPENSE mode:

```cpp
// tracking_manager.cpp line ~423
if(planningMode == planningType::DISPENSE) {
    v_ctrl = -0.5 * v_ctrl;  // reverse velocity to escape
}
```

This directly causes **sign-flipping in cmd_vel**.  The sequence:
1. Incomplete obstacle data causes the robot to slow down near undetected obstacles
2. Speed falls below 0.8 m/s for 1.2 seconds → DISPENSE mode activates
3. `v_ctrl = -0.5 * v_ctrl` → velocity reverses
4. After 0.5s in DISPENSE, `replan_now = true` → global replan triggered
5. New trajectory computed → exits DISPENSE → forward again
6. Same obstacles reappear → cycle repeats

### Cause 3: Unreachable Collision Replan Threshold

```cpp
// tracking_manager.cpp checkReplanFlag()
replan_check_flag window size: 25 frames  (in rcvLidarIMUPosCallback)
threshold: num_collision_error > 100       // NEVER reachable (max = 25)
```

The comment in the code says "实车参数为15" (real-robot parameter should be 15)
but the threshold was left at 100 — a value only meaningful for the simulation
path (250-frame window).  This means the robot could **never** trigger a
collision-based replan, relying entirely on `tracking_low > 6` and `replan_now`.

Without proper collision-based replanning, the robot sometimes continues towards
detected obstacles for too long before the `tracking_low` mechanism kicks in,
leading to abrupt last-moment replans.

### Cause 4: Tracking-Low Replan Trigger

In `local_planner.cpp`, when the robot strays > 0.4m from the reference
trajectory, `tracking_low_check_flag` is set to 1.  With a window of 10 and
threshold of 6, if the robot is off-track for 6/10 consecutive cycles, a replan
is forced.

The localization tilt (from IMU mismatch) causes systematic position error,
making the robot appear off-track even when physically following the path →
frequent tracking_low replans.

## Fixes Applied

### Fix A: Filter LiDAR Topic Bug (launch param)

**File:** `Navigation-filter-test/ws_cloud/src/livox_cloudpoint_processor/launch/lidar_filter_pointcloud.launch`

```xml
<!-- BEFORE -->
<param name="scan_topic_right" value="/livox/lidar_192_168_1_105" />
<!-- AFTER -->
<param name="scan_topic_right" value="/livox/lidar_192_168_1_3" />
```

Restores full dual-LiDAR obstacle detection.  **This is the single most
impactful fix** — with complete obstacle coverage:
- Planners see all obstacles → no routing through real barriers
- MPC has stable obstacle data → smooth velocity output
- Robot doesn't get "stuck" on undetected obstacles → less DISPENSE triggering

### Fix B: Collision Replan Threshold (C++ change + rebuild)

**File:** `HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_manager.cpp`

```cpp
// BEFORE
if(num_collision_error > 100 || ...)  // never reachable with 25-frame window

// AFTER
if(num_collision_error > 15 || ...)   // real-robot parameter as per original comment
```

Now with 25-frame window and threshold 15: if > 60% of recent frames show
collision, the robot replans early rather than waiting for tracking_low or
DISPENSE to trigger.  This produces **earlier, smoother replanning** instead of
abrupt last-moment corrections.

**Rebuilt:**
```bash
cd /home/sentry_train_test/AstarTraining/HIT_code/sentry_planning_ws
catkin_make --pkg tracking_node
```

### Fix C: IMU Unification (Indirect)

The localization tilt fix (separate report) reduces systematic position error,
which in turn reduces tracking_low triggers, which reduces unnecessary replans.

## Replan Trigger Summary (After Fixes)

| Trigger | Window | Threshold | Typical Cause | Expected Frequency |
|---------|--------|-----------|---------------|-------------------|
| `num_collision_error > 15` | 25 frames | 15 (60%) | Obstacle on planned path | Rare (with fixed filter) |
| `num_tracking_low > 6` | 10 frames | 6 (60%) | Robot > 0.4m off trajectory | Occasional |
| `replan_now` (near goal) | — | target_dist < 1.0m | Pre-emptive near arrival | Normal |
| `replan_now` (DISPENSE) | — | stuck > 0.5s | Robot physically stuck | Rare |

## Verification

```bash
# Monitor cmd_vel for sign stability
rostopic echo /cmd_vel/linear --noarr | head -20

# Check replan frequency — should be infrequent
rostopic echo /replan_flag/data

# Monitor tracking quality — should show few "tracking time low!" warnings
rosnode info /tracking_node | grep -c "tracking time low"
```

Expected after fixes:
- cmd_vel should maintain consistent sign (positive when moving towards goal)
- Global replans should only occur near obstacles or goal changes
- DISPENSE mode ("卡住你就倒车拐弯") should rarely activate

---

## Fix 34 Update (MPC Oscillation — 3 additional root causes)

Three additional MPC oscillation mechanisms were identified and fixed in Fix 34:

1. **`speed_direction` flip-flop** (Fix 34b): No hysteresis at ±π/2 boundary caused every-frame sign reversal of the entire reference trajectory → violent warm-start corruption. Fixed with 63°–108° hysteresis band.

2. **Time-reset chain reaction** (Fix 34c): 0.4m threshold triggered every frame → repeated `linearation()` calls → stale warm-starts → accumulated `tracking_low_check_flag` → unnecessary replans. Fixed with 0.6m threshold + 0.3s cooldown.

3. **Stale warm-start on new trajectory** (Fix 34d): `rcvGlobalTrajectory()` reset time but not solver state → first frames used warm-start from completely different trajectory. Fixed with `mpcSolverPtr_->reset()` on trajectory receive.

See Fix 34 in PLANNING_FIX_HISTORY.md for full details.

---

## Fix 35 Update — Final Resolution (Oscillation Eliminated)

Fix 34 reduced but did not eliminate oscillation. Three deeper root causes were found via live diagnostics:

1. **Observation state corruption** (Fix 35b, CRITICAL): `state(2) = speed_direction * state(2)` negated the MPC observation speed when `speed_direction == -1`. The solver received contradictory state info (thinking the robot was moving backward), producing overcorrection that flipped signs every frame. **Fixed by removing** this line — the solver now always sees the true physical speed.

2. **DISPENSE vicious loop** (Fix 35d): With MPC oscillation producing near-zero net speed, `checkMotionNormal()` (max < 0.3, avg < 0.1, 1.2s) triggered DISPENSE mode, which reversed `v_ctrl = -0.5 * v_ctrl`, worsening the oscillation. **Fixed** by tightening thresholds: max < 0.15, avg < 0.05, 2.0s window.

3. **Time-reset threshold still too tight** (Fix 35c): Live echo showed `dis=0.628` still triggering the 0.6m threshold from Fix 34c. **Raised to 0.8m.**

**Result:** Over 15s of live testing, only 2/145 cmd_vel samples were negative (tiny: -0.02), compared to every other sample flipping between -0.2 and +0.4 before. No DISPENSE or time-reset triggers observed.

**Status: ✅ RESOLVED** — See Fix 35 in PLANNING_FIX_HISTORY.md for details.

---

## Fix 37 Update — Frame Mismatch Between tracking_node and trajectory_generation

After undoing a broken intermediate fix (Fix 36e/f), the reference path appeared 7.7 m ahead of the predicted path in RViz. Root cause: **two compounding issues:**

1. **Stale binary** — user undid source edits but the running binary was from a prior (broken) compilation. Rebuilding alone would have fixed the 7.7 m divergence.

2. **LiDAR-to-chassis offset mismatch** — `tracking_manager.cpp` applied a rotated (-0.011, -0.17166) m offset to convert LiDAR position to chassis centre, but `replan_fsm.cpp` (trajectory_generation) used the raw LiDAR position. The MPC observation was in a different frame from the reference trajectory — a systematic ~0.17 m error that rotated with yaw.

**Fix:** Removed the LiDAR-to-chassis offset from `tracking_manager.cpp`. Both packages now use raw `aft_mapped` (LiDAR) coordinates consistently.

**Result (15s live test):**
- Ref↔Pred distance: 7.7 m → 0.07 m
- Mean speed: 0.501 m/s, max 0.819 m/s
- Zero-speed: 0.7%
- vx sign flips: 2
- "MPC is not safe": 92/10s → 19/10s (79% reduction)

**Status: ✅ RESOLVED** — See Fix 37 in PLANNING_FIX_HISTORY.md for details.

---

## Fix 38 Update — Replan, Deviation, Startup Speed, Obstacle Braking

Four additional issues addressed in Fix 38:

1. **No replan after obstacle clears** (Fix 38b): Added 5s periodic replan timer. When a dynamic obstacle moves away, the robot no longer stays on the old detour path indefinitely — trajectory_generation re-evaluates every 5s and finds shorter routes through cleared areas.

2. **No replan after veer off course** (Fix 38a): `tracking_low_check_flag` was always 0, disabling the `num_tracking_low > 4` replan trigger. Now checks `min_dist > 0.8m` (distance from robot to nearest reference point) — if the robot is off-course for 5+ of 10 frames, replan fires.

3. **Very slow startup** (Fix 38c): `MAX_LEAD_TIME = 1.0s` limited MPC look-ahead too aggressively — the robot plateaued at ~0.05 m/s for 7+ seconds before moving. Raised to 2.0s so the MPC has sufficient trajectory look-ahead to command aggressive acceleration from frame 1.

4. **Too fast near obstacles** (Fix 38d): Added obstacle-proximity speed reduction in `getFightTrackingTraj()`. When obstacles from the MPC obstacle search are within 0.8m of a reference point, `ref_speed` is scaled down to 35% floor. This gives feed-forward deceleration instead of relying solely on the reactive collision barrier.

**Result (15s live test):**
- Time to 0.3 m/s: 7.32s → **0.00s** (instant)
- Mean speed: 0.398 → **0.664 m/s** (+67%)
- Min speed: 0.002 → 0.416 m/s (no stalling)
- vx sign flips: 0
- Periodic replan: fires every 5s ✅

**Status: ✅ RESOLVED** — See Fix 38 in PLANNING_FIX_HISTORY.md for details.
