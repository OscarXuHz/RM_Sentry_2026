# DISPENSE Mode — Technical Mechanism Report

## Overview

DISPENSE (摆脱模式, "escape mode") is a recovery behavior that activates when the robot
detects it is physically stuck — unable to make forward progress despite having a valid
trajectory.  When triggered, DISPENSE **reverses the MPC velocity command** for a brief
period, then requests a global replan to find an alternate route.

Source: [`tracking_manager.cpp`](../../HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_manager.cpp),
[`tracking_manager.h`](../../HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/include/tracking_manager.h).

---

## 1. Mode Enum

```cpp
// tracking_manager.h, line 34
typedef enum PLANNING_TYPE {
    FASTMOTION = 1,   // 快速移动模式 (normal forward motion)
    DISPENSE   = 2    // 摆脱模式 (escape/unstuck mode)
} planningType;
```

The system starts in `FASTMOTION` and transitions to `DISPENSE` only when stuck
detection triggers.

---

## 2. Stuck Detection: `checkMotionNormal()`

Called every tracking cycle (when `reference_path.size() > 2`), this function monitors
the robot's speed history and triggers DISPENSE when motion stalls.

```cpp
// tracking_manager.cpp, line 815
void tracking_manager::checkMotionNormal(double line_speed)
{
    // New trajectory received → reset everything
    if (localplanner->m_get_global_trajectory) {
        planningMode = planningType::FASTMOTION;
        replan_now = false;
        start_checking_time = ros::Time::now();
        speed_check.clear();
    }

    speed_check.insert(speed_check.begin(), line_speed);
    double tracking_time = (ros::Time::now() - start_checking_time).toSec();

    if (tracking_time > 2.0) {
        speed_check.pop_back();
        double max_speed = *max_element(speed_check.begin(), speed_check.end());
        double avg_speed = accumulate(...) / speed_check.size();

        if (max_speed < 0.15 && !arrival_goal && avg_speed < 0.05) {
            if (planningMode != planningType::DISPENSE) {
                dispense_time = ros::Time::now();  // start timer
            }
            planningMode = planningType::DISPENSE;
        }
    }
}
```

### Trigger Conditions (ALL must be true)

| Condition | Threshold | Purpose |
|-----------|-----------|---------|
| `tracking_time > 2.0` | 2.0 s | Must have been tracking for ≥2 seconds |
| `max_speed < 0.15` | 0.15 m/s | No speed sample in window exceeded this |
| `avg_speed < 0.05` | 0.05 m/s | Average speed is near zero |
| `!arrival_goal` | — | Not already at the destination |

These thresholds were tightened by **Fix 35d** (was 1.2 s / 0.30 / 0.10).  The old
thresholds triggered during normal MPC transients, creating a feedback loop where
DISPENSE reversal amplified oscillation → stuck detection fired again → perpetual escape.

---

## 3. DISPENSE Behavior: Velocity Reversal

When DISPENSE is active, the MPC velocity output is reversed and halved:

```cpp
// tracking_manager.cpp, line 494 (LiDAR path)

// Fix 45c skips negative-v clamping in DISPENSE mode
if (v_ctrl < 0.0 && planningMode != planningType::DISPENSE) {
    v_ctrl = 0.0;
}

if (planningMode == planningType::DISPENSE) {
    double reverse_time = (ros::Time::now() - dispense_time).toSec();
    if (reverse_time > 0.5) {
        replan_now = true;   // request new trajectory after 0.5s
    }
    v_ctrl = -0.5 * v_ctrl;  // 卡住你就倒车拐弯 ("stuck → reverse and turn")
}
```

### Effect on Robot Motion

1. **MPC computes** `v_ctrl > 0` (forward toward reference path)
2. **DISPENSE multiplies** by −0.5 → robot moves backward at half speed
3. **MPC heading** (`phi_ctrl`) is preserved → robot turns while reversing
4. After **0.5 s** of reversing, `replan_now = true` → global replan requested
5. New trajectory arrives → mode resets to `FASTMOTION`

The 0.5 s reversal window is short enough to back away from the stuck position without
overshooting, but long enough to create clearance for the replanner to find a new route.

### Gazebo Path Variant

The Gazebo callback has the same logic but with a slightly longer timeout (**0.6 s**
instead of 0.5 s) and no negative-v clamping bypass (Fix 45c only applies to the LiDAR
path).

---

## 4. DISPENSE Exit Sequence

DISPENSE exits through `checkReplanFlag()`:

```
  DISPENSE active
       │
       ├─ After 0.5s → replan_now = true
       │
       ▼
  checkReplanFlag() detects: replan_now == true
       │
       ├─ planningMode = FASTMOTION      ← mode reset
       ├─ replan_flag.data = true         ← publish to /replan_flag
       │
       ▼
  trajectory_generation receives replan request
       │
       ├─ pathFinding() → new topo search → new trajectory
       │
       ▼
  New trajectory arrives at tracking_node
       │
       ├─ m_get_global_trajectory = true
       ├─ checkMotionNormal resets: FASTMOTION, clear speed_check
       └─ Normal tracking resumes
```

---

## 5. Interaction with Other Systems

### Fix 45c (Negative Velocity Clamping)

In `FASTMOTION` mode, negative `v_ctrl` is clamped to 0 to prevent backward motion.
DISPENSE is explicitly exempted from this clamp — it needs negative velocity to reverse:

```cpp
if (v_ctrl < 0.0 && planningMode != planningType::DISPENSE) {
    v_ctrl = 0.0;
}
```

### Motion Mode (`checkMotionMode`)

When DISPENSE is active, the chassis spinning mode (`xtl_mode`) is suppressed:

```cpp
// tracking_manager.cpp, line 728
if (time > 0.5 && !checkBridge() && planningMode != planningType::DISPENSE) {
    xtl_mode = 0;  // normal forward driving
}
```

This prevents the chassis from spinning while the robot is reversing.

### Fix 42c (Collision Replan Cooldown)

The 3-second collision-replan cooldown does NOT apply to DISPENSE-triggered replans.
`replan_now = true` bypasses the cooldown check because it's classified as a non-collision
trigger:

```cpp
bool is_collision_only_trigger = (num_collision_error > 25)
                               && !(num_tracking_low > 4)
                               && !replan_now && !periodic_replan;
if (is_collision_only_trigger && !collision_replan_ok) {
    return;  // suppress collision-only replan during cooldown
}
// DISPENSE replan (replan_now=true) passes through
```

---

## 6. Complete Lifecycle Timeline

```
t=0.0s  Robot tracking trajectory normally (FASTMOTION)
        Speed: ~1.0 m/s
        ↓
t=X.0s  Robot hits physical obstacle / gets wedged
        Speed drops to ~0.02 m/s
        checkMotionNormal accumulates low-speed window
        ↓
t=X+2.0s  Stuck detected: max<0.15, avg<0.05
          planningMode = DISPENSE
          dispense_time = now
          ↓
t=X+2.0s  v_ctrl reversed: robot backs up at ~0.5× forward speed
  to       MPC heading still tracking → robot turns while reversing
t=X+2.5s
          ↓
t=X+2.5s  reverse_time > 0.5s → replan_now = true
          checkReplanFlag publishes replan request
          planningMode = FASTMOTION
          ↓
t=X+2.6s  trajectory_generation receives replan, runs pathFinding()
          New trajectory avoids the stuck location
          ↓
t=X+2.7s  New trajectory arrives at tracking_node
          checkMotionNormal resets speed history
          Robot resumes forward motion on new path
```

---

## 7. Known Issues and Considerations

1. **Test environment false trigger**: When the robot is stationary on a bench (not
   actually stuck), `checkMotionNormal` will trigger DISPENSE after 2 seconds because
   `avg_speed ≈ 0`.  This is expected behavior in the test environment but should not
   occur during competition when the robot is actively moving.

2. **MPC cold start after replan**: After DISPENSE triggers a replan, the new trajectory
   arrives and MPC restarts cold.  The first few MPC cycles may produce low or zero
   velocity, potentially re-triggering DISPENSE if the 2-second window passes quickly.
   The reset of `start_checking_time` on new trajectory receipt prevents immediate
   re-trigger.

3. **Velocity magnitude**: The `−0.5` factor means the maximum reverse speed is half the
   MPC's forward command.  With typical MPC output of 1.0–1.5 m/s, the reverse speed
   caps at 0.5–0.75 m/s, which provides a safe escape velocity.
