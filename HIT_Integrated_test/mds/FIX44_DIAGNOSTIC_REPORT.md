> **⚠️ HISTORICAL** — This report predates Fixes 45–48.  Symptoms described here (path divergence, straight-line reference) have been addressed by later fixes.  See [PLANNING_FIX_HISTORY.md](PLANNING_FIX_HISTORY.md) for the full chronology.

# Fix 44 — Diagnostic Report: Path Divergence, Obstacle Avoidance, and Global Planning

**Date**: 2025-01-XX  
**System**: RM Sentry 2026 — HIT SLAM+Navigation  
**Context**: System ported from another robot to new omnidirectional platform  
**Status**: Fixes implemented, compiled successfully, pending live test

---

## 1. Summary of Reported Problems

After Fix 38–43, three critical issues persisted:

| # | Problem | Severity | When |
|---|---------|----------|------|
| 1 | Predicted path diverges and fluctuates | High | Corners especially; improved after Fix 43 but still severe |
| 2 | Does not avoid walls and obstacles | Critical | Both static (map) and dynamic (live point cloud) |
| 3 | Robot often fails to plan global trajectory | Medium | Intermittent; "often" suggests recurrent |

---

## 2. Live Diagnostic Observations (Pre-Fix 44)

Topic echoes from the running system revealed:

| Topic | Status | Value |
|-------|--------|-------|
| `/odom` | Publishing | pos=(-3.43, -5.01), within map bounds |
| `/cmd_vel` | Publishing | vx≈0.5–0.6, vy≈0.4–0.6, **angular.z=0.0 always** |
| `/sentry_des_speed` | Silent (timeout) | Possible echo timing issue; hit_bridge IS forwarding |
| `/solver_status` | Publishing | data=True (solver converging) |
| `/tracking/mpc_predicted_path` | Publishing | 20 poses (full MPC horizon) |
| `/trajectory_generation/global_trajectory` | Silent | No new trajectory being published |
| `/replan_flag` | Silent | Not being published (see Issue 3 analysis) |
| `/clicked_point` | Silent | No goal from decision tree observed |
| `/slaver/wheel_state` | Silent | Wheel encoder not publishing (MCU may be off) |

**Key observation**: `angular.z=0.0` is expected — the robot is omnidirectional, receiving world-frame vx/vy. The MCU converts these to wheel speeds internally. Chassis rotation is controlled by `xtl_flag`, not angular velocity.

---

## 3. Root Cause Analysis

### 3.1 Issue 1: Predicted Path Diverges (Especially at Corners)

**ROOT CAUSE: MPC φ state uses chassis heading instead of velocity heading**

The MPC kinematic model is:
```
dx/dt = v·cos(φ)
dy/dt = v·sin(φ)
v̇ = a
φ̇ = ω
```

This model assumes **φ is the velocity direction** (the direction the robot is moving). But the code observed φ from `robot_cur_yaw` = **chassis heading** (wheel encoder or LiDAR orientation).

**For an omnidirectional robot, chassis heading ≠ velocity direction.** The robot can move sideways, diagonally, or in any direction without changing chassis orientation. When the MPC observes chassis heading as its φ state, it predicts incorrect future positions, causing divergence.

**Example**: Robot moves East (velocity heading = 0°), chassis faces North (chassis heading = 90°). MPC thinks robot is moving North → plans correction → outputs wrong velocity → actual motion contradicts prediction → divergence escalates.

This is especially severe at corners because:
1. The velocity direction changes rapidly while chassis may lag
2. The reference heading (from planned trajectory) changes sharply
3. The mismatch between chassis heading and velocity direction is maximized

**Secondary cause**: The integer-division yaw wrapping had a discontinuity at ±π:
```cpp
int k = robot_yaw_temp / (2 * M_PI);   // C++ truncates toward zero
int j = robot_yaw_temp / M_PI;          // discontinuity at exactly ±π
```
When heading crossed π, `robot_cur_yaw` jumped by ~2π, corrupting the solver's warm-start and producing 1-2 frames of garbage output.

**Speed observation error**: When `has_wheel_state_=true`, speed was decomposed as:
```cpp
robot_cur_speed(0) = robot_wheel_speed * cos(robot_cur_yaw);
robot_cur_speed(1) = robot_wheel_speed * sin(robot_cur_yaw);
```
This projected scalar wheel speed along chassis heading — correct for differential drive, **wrong for omnidirectional**. The MPC saw incorrect velocity components.


### 3.2 Issue 2: Does Not Avoid Walls and Obstacles

**ROOT CAUSE A: Weak barrier penalty relative to tracking cost**

The collision barrier strength was μ=20, δ=1.0. At the constraint boundary (h=0, distance = √d_threshold from obstacle):
- Barrier gradient per obstacle: μ · 2 · dist / (h+δ) = 20 · 2 · 0.5 / 1.0 = **20**
- Tracking cost gradient at 0.5m deviation: Q · deviation = 150 · 0.5 = **75**

**With a single nearby obstacle, the tracking cost (75) beats the barrier (20) by 3.75×.** The MPC prefers to violate the obstacle boundary rather than deviate from the reference.

With 3-4 obstacles per step (typical for a wall), total barrier push ≈ 80 vs tracking pull ≈ 75 — barely balanced. Near corners with fewer obstacles, tracking dominates.

**ROOT CAUSE B: Collision distance thresholds too tight**

- Static threshold: d² = 0.25 → effective clearance = 0.50m
- Robot radius: 0.35m
- Safety margin: only 0.15m (one map cell at 0.05m resolution)

This 0.15m margin is too thin for a real robot with:
- Localization uncertainty (~0.05m)
- Control latency (~50ms at 1 m/s → 0.05m)
- Wheel slip and inertia

**ROOT CAUSE C (contributing): Divergence amplifies avoidance failure**

The MPC divergence from Issue 1 compounds the obstacle problem. When the predicted path diverges from the reference, obstacles between them are invisible (the search only covers ±1.0m around the reference). The diverged path can pass straight through obstacles that weren't in the search window.


### 3.3 Issue 3: Robot Often Fails to Plan Global Trajectory

**ROOT CAUSE A: `arrival_goal` startup deadlock**

`arrival_goal` is initialized to `true` in the header:
```cpp
bool arrival_goal = true;
```

When `arrival_goal=true`, the `rcvLidarIMUPosCallback` returns early without:
1. Running the MPC solver
2. Calling `checkReplanFlag()` → `/replan_flag` goes silent
3. Only `publishSentryOptimalSpeed(zeros)` runs

To exit this state, `m_get_global_trajectory` must become true (set when `rcvGlobalTrajectory` callback fires). But if the decision tree hasn't sent a goal, no trajectory gets generated → deadlock.

**ROOT CAUSE B: `/replan_flag` silent during arrival state**

The early return bypassed `checkReplanFlag()` entirely. This meant:
- The periodic replan (Fix 38b, every 5s) never triggered during arrival
- External monitors couldn't distinguish "at goal, waiting" from "node crashed"
- trajectory_generation had no signal to replan even if conditions changed

**ROOT CAUSE C: Decision tree dependency**

The first global trajectory requires the decision tree to publish a goal/waypoint. If:
- The decision tree node isn't running
- The strategy tree is in a wait state
- The team color detection failed (robot_status not published)

No initial trajectory gets generated. The system is fully dependent on the behavior tree proceeding correctly.

---

## 4. Fixes Implemented (Fix 44a–e)

### Fix 44a: Velocity Heading for MPC State (tracking_manager.cpp)

**The core fix for divergence.** Replaced chassis heading with velocity direction computed from LiDAR odom twist:

```cpp
// Compute velocity heading from actual odom velocities
double odom_vx = twist.linear.x;
double odom_vy = twist.linear.y;
double odom_speed = std::hypot(odom_vx, odom_vy);

double raw_heading;
if (odom_speed > 0.2) {
    raw_heading = std::atan2(odom_vy, odom_vx);  // true velocity direction
} else {
    raw_heading = robot_lidar_yaw;  // fallback at low speed
}

// Maintain continuous heading (no ±π wrapping discontinuity)
if (!velocity_heading_init_) {
    velocity_heading_ = raw_heading;
    velocity_heading_init_ = true;
} else {
    double delta = std::remainder(raw_heading - velocity_heading_, 2.0 * M_PI);
    velocity_heading_ += delta;
}

robot_cur_yaw = velocity_heading_;
```

Also changed speed observation to always use LiDAR odom twist (not wheel encoder decomposed along chassis heading).

Velocity decomposition simplified — no `robot_add_yaw` unwrapping needed:
```cpp
MPC_Control(0) = v_ctrl * std::cos(phi_ctrl);  // direct world-frame vx
MPC_Control(1) = v_ctrl * std::sin(phi_ctrl);  // direct world-frame vy
```

**Why this works**: The MPC model `dx/dt = v·cos(φ)` now holds exactly, because φ IS the velocity direction (from atan2(vy,vx)) and v is the speed magnitude (from hypot(vx,vy)). The predicted position matches reality → no divergence.


### Fix 44b: Increased Barrier Strength μ (local_planner.cpp)

Changed `ocs2::RelaxedBarrierPenalty` from μ=20 to **μ=40**.

New gradient analysis:
- Single obstacle barrier gradient at 0.5m: 40 · 2 · 0.5 / 1.0 = **40**
- Tracking gradient at 0.5m: 150 · 0.5 = **75**
- 2 obstacles: 80 vs 75 → **barrier wins**
- 3 obstacles (typical wall): 120 vs 75 → barrier strongly dominates

The stronger barrier ensures the MPC respects obstacle clearance even in geometries with sparse obstacles (corners, isolated pillars).


### Fix 44c: Increased Collision Distance Thresholds (SentryRobotCollisionConstraint.h)

| Parameter | Old | New | Effective Clearance | Margin (0.35m robot) |
|-----------|-----|-----|--------------------|-----------------------|
| `distance_threshold_` (static) | 0.25 | **0.36** | 0.60m | 0.25m |
| `distance_threshold_dynamic_` | 0.49 | **0.64** | 0.80m | 0.45m |

The barrier influence radius (where penalty becomes active) is now:
- Static: √(0.36 + 1.0) = **1.17m** (was 1.12m)
- Dynamic: √(0.64 + 1.0) = **1.28m** (was 1.22m)


### Fix 44d: Replan Flag During Arrival State (tracking_manager.cpp)

Added `global_replan_pub.publish(false)` in the `arrival_goal` early-return branch. This ensures:
- `/replan_flag` topic stays alive even when the robot is at its goal
- External monitors can distinguish "at goal" from "node crashed"
- Periodic replan logic can still trigger when conditions change


### Fix 44e: Debug Diagnostics (tracking_manager.cpp)

Added periodic ROS_INFO every ~1s:
```
[Fix44] velHead=X.XXX lidarYaw=X.XXX odomSpd=X.XXX vx=X.XXX vy=X.XXX hasWheel=N
```

This allows real-time monitoring of:
- Velocity heading vs chassis heading discrepancy
- Speed source and magnitude
- Whether wheel encoder is active

Combined with existing Fix 43 diagnostics in `solveNMPC()`:
```
[Fix43] obs=N min_obs=X.XXXm@sN maxDev=X.XXX/X.XXXm refGap=X.XXXm spd0=X.XX v=X.XX pos=[X.XX,X.XX]
```

---

## 5. Files Modified

| File | Changes |
|------|---------|
| `trajectory_tracking/include/tracking_manager.h` | Added `velocity_heading_`, `velocity_heading_init_` members |
| `trajectory_tracking/src/tracking_manager.cpp` | Fix 44a: velocity heading, speed observation, simplified decomposition; Fix 44d: replan flag in arrival state; Fix 44e: debug logging |
| `trajectory_tracking/src/local_planner.cpp` | Fix 44b: barrier μ 20→40, updated log message |
| `trajectory_tracking/include/ocs2_sentry/constraint/SentryRobotCollisionConstraint.h` | Fix 44c: distance thresholds increased |

---

## 6. Testing Protocol

### 6.1 Pre-Launch Checks
```bash
# Verify build
cd ~/AstarTraining/RM_Sentry_2026/HIT_code/sentry_planning_ws
catkin_make --pkg tracking_node

# Check map files exist
ls $(rospack find trajectory_generation)/map/{occfinal.png,bevfinal.png,occtopo.png}
```

### 6.2 Launch and Monitor
```bash
# Launch full system
roslaunch ... HIT_intergration_test.launch

# Monitor in separate terminals:
rostopic echo /cmd_vel                                # vx/vy should be non-zero when moving
rostopic echo /solver_status                          # should be True
rostopic echo /replan_flag                            # should publish (True or False)
rostopic echo /tracking/mpc_predicted_path            # 20 poses, should not diverge

# Watch for Fix 44 diagnostics:
rostopic echo /rosout -p | grep "Fix44\|Fix43"
```

### 6.3 Verification Criteria

**Issue 1 (Divergence)**:
- `[Fix44] velHead` should be close to `lidarYaw` when moving straight
- `[Fix43] maxDev` near-term should be < 0.3m (was 2+ meters)
- Predicted path on RViz should follow reference closely at corners
- `velHead` vs `lidarYaw` may diverge during spins (this is correct — they're different quantities)

**Issue 2 (Obstacle Avoidance)**:
- `[Fix43] min_obs` should stay > 0.35m (robot radius)
- No "MPC is not safe" warnings in steady state
- Path should visibly curve around obstacles in RViz

**Issue 3 (Global Planning)**:
- `/replan_flag` should always publish (False when at goal, True when replanning)
- After decision tree sends first goal, `/trajectory_generation/global_trajectory` should publish

---

## 7. Risk Assessment

| Fix | Risk | Mitigation |
|-----|------|------------|
| 44a (velocity heading) | At low speed, velocity heading is noisy; fallback to LiDAR yaw may cause brief jump | Threshold at 0.2 m/s; continuous heading smoothing via `std::remainder` |
| 44a (always odom speed) | If LiDAR odom has transient spikes, MPC sees wrong speed | LiDAR odom (hdl_localization UKF) already smoothed; more reliable than raw wheel encoder |
| 44b (stronger barrier) | μ=40 may make MPC too conservative, slowing in tight corridors | Only 2× increase; tracking Q=150 still dominates in open areas; can reduce to 30 if too aggressive |
| 44c (larger distance) | 0.36 threshold may trigger avoidance too far from walls, limiting corridor navigation | Effective clearance 0.6m — still narrower than most corridors; map inflation already accounts for robot radius |
| 44d (replan in arrival) | Publishing replan_flag=false when at goal could trigger stale monitoring logic | Only publishes false, not true — doesn't trigger replanning |

---

## 8. Remaining Concerns

### 8.1 Wheel Encoder Dependency
`/slaver/wheel_state` was silent during diagnostics. If the MCU is not connected or not sending feedback, `has_wheel_state_` stays false. Fix 44a removes dependency on wheel encoder for MPC state, but other subsystems (motion mode detection, stuck detection) may still be affected.

### 8.2 Decision Tree Startup
The first global trajectory depends on the decision tree sending a goal. If the behavior tree doesn't start (e.g., team color not detected because `/slaver/robot_status` is silent), the robot will stay in `arrival_goal=true` indefinitely. Consider adding a "default patrol" trajectory as fallback.

### 8.3 MPC Model Limitation
The unicycle model [x,y,v,φ] with Fix 44a works when φ = velocity direction. But during pure rotation (spinning in place at xtl_mode=2,3), the robot has zero translational velocity → velocity heading undefined → falls back to LiDAR heading. If the robot transitions from spin to translation, there may be a brief heading adjustment period. Monitor `[Fix44]` logs for this case.

### 8.4 Map Frame Consistency
The reference trajectory is in the LiDAR/aft_mapped frame. Fix 44a uses odom twist (also in this frame) for velocity heading. If there's a TF inconsistency between `/odom_local` (remapped from `/odometry_imu`) and the map frame, the heading would have a constant offset. Verify TF tree after launch.

---

## 9. Quick Reference: Parameter Summary (Post-Fix 44)

| Parameter | Value | Location |
|-----------|-------|----------|
| Barrier μ | 40 | local_planner.cpp (linearation) |
| Barrier δ | 1.0 | local_planner.cpp (linearation) |
| Static collision d² | 0.36 | SentryRobotCollisionConstraint.h |
| Dynamic collision d² | 0.64 | SentryRobotCollisionConstraint.h |
| MPC φ source | velocity heading (atan2 of odom twist) | tracking_manager.cpp |
| Speed source | LiDAR odom twist magnitude | tracking_manager.cpp |
| Low-speed fallback | 0.2 m/s → LiDAR heading | tracking_manager.cpp |
| Q matrix | [150, 150, 60, 80] | task.info |
| R matrix | [1.5, 0.15] | task.info |
| Obstacle search half | 20 cells (±1.0m) | local_planner.cpp |
| Sectors per step | 8 | local_planner.cpp |
| MAX_LEAD_TIME | 0.5s | local_planner.cpp |
| HPIPM mode | ROBUST, reg_prim=1e-8 | local_planner.cpp |
| SQP iterations | 12 | task.info |
| Robot radius | 0.35m | trajectory_planning.launch |
