# Parameter Tuning Guide

This document shows **exactly where** you can adjust all key tuning parameters in your system.

---

## 1. **Collision Penalty & Barrier Smoothness**

### File: [local_planner.cpp](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/local_planner.cpp#L179)
### Location: Line 179

```cpp
ocs2::RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(2.0, 0.1);
```

- **`mu` (first parameter) = `2.0`** — Collision penalty weight
  - Higher → stronger obstacle repulsion
  - Lower → robot may cut closer to obstacles
  - Range: **0.5–10**
  - **Adjust**: Change `2.0` to your desired value

- **`delta` (second parameter) = `0.1`** — Barrier smoothness zone width
  - Smaller → sharper but more aggressive avoidance
  - Larger → smoother but weaker repulsion
  - Range: **0.01–0.5**
  - **Adjust**: Change `0.1` to your desired value

---

## 2. **Distance Threshold (Obstacle Clearance)**

### File: [SentryRobotCollisionConstraint.h](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/include/ocs2_sentry/constraint/SentryRobotCollisionConstraint.h#L37)
### Location: Line 37

```cpp
ocs2::scalar_t distance_threshold_ = 0.36;  // was 0.25 (0.5m), now 0.6m from inflated obstacle center
```

- **`distance_threshold_` = `0.36` m** — Minimum clearance from obstacles
  - Higher values → safer but more conservative paths
  - Lower values → robot cuts closer to obstacles
  - **Adjust**: Change `0.36` to your desired value

---

## 3. **Barrier Penalty Parameters in SQP (state-input bounds)**

### File: [task.info](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/cfg/task.info#L1-L15)
### Location: Lines 1–15

```plaintext
sqp
{
  nThreads                              2
  dt                                    0.1
  sqpIteration                          10
  inequalityConstraintMu                0.1      ← penalty for exceeding bounds
  inequalityConstraintDelta             0.2      ← smoothness zone
  useFeedbackPolicy                     true
  projectStateInputEqualityConstraints  true
  printSolverStatistics                 false
  printSolverStatus                     false
  printLinesearch                       false
  useFeedbackPolicy                     true
  integratorType                        RK2
  threadPriority                        50
}
```

- **`inequalityConstraintMu` = `0.1`** — Penalty for violating velocity/acceleration limits
  - Higher → enforces harder constraints
  - Lower → allows slight violations
  - **Adjust**: Change `0.1` to your desired value

- **`inequalityConstraintDelta` = `0.2`** — Smoothness zone width for constraint barriers
  - **Adjust**: Change `0.2` to your desired value

---

## 4. **State Tracking Weight Matrix Q**

### File: [task.info](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/cfg/task.info#L36-L44)
### Location: Lines 36–44

```plaintext
Q
{
    (0,0)   80.0    ; x position weight
    (1,1)   80.0    ; y position weight
    (2,2)   15.0    ; velocity weight
    (3,3)   70.0    ; yaw (heading) weight
}
```

**What it does**: Penalizes deviation from reference trajectory
- **x/y position weights (80.0 each)** → Higher = tighter path tracking, no cuts across curves
- **Velocity weight (15.0)** → Controls speed tracking; was `0.0` before
- **Yaw weight (70.0)** → Controls heading alignment

**Adjust by changing any diagonal value** to increase/decrease that state's importance

---

## 5. **Input Cost Weight Matrix R**

### File: [task.info](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/cfg/task.info#L46-L50)
### Location: Lines 46–50

```plaintext
R
{
    (0,0)   1.5     ; acceleration weight
    (1,1)   0.1     ; angular rate weight
}
```

**What it does**: Penalizes control effort and jerk
- **Acceleration weight (1.5)** → Higher = smoother acceleration profile, less jerky motion
- **Angular rate weight (0.1)** → Controls turning smoothness

**Adjust by changing any diagonal value** to increase/decrease that input's cost

---

## 6. **Global Planner Reference Speed**

### File: [HIT_intergration_test.launch](HIT_Integrated_test/HIT_intergration_test.launch#L11)
### Location: Line 11

```xml
<arg name="reference_desire_speed" default="2.8" />
```

- **`reference_desire_speed` = `2.8` m/s** — Target cruise speed for global planner
  - **Adjust**: Change `2.8` to your desired speed
  - Used throughout the planning pipeline

---

## 7. **Robot Radius (Obstacle Inflation)**

### File: [HIT_intergration_test.launch](HIT_Integrated_test/HIT_intergration_test.launch#L10)
### Location: Line 10

```xml
<arg name="robot_radius" default="0.35" />
```

- **`robot_radius` = `0.35` m** — Physical robot radius used for obstacle inflation
  - Higher values → obstacles "grow" more, causing larger detours
  - Lower values → robot cuts closer to obstacles
  - **Adjust**: Change `0.35` to your desired value

---

## 8. **Local Planner Velocity & Acceleration Limits**

### File: [trajectory_planning.launch](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/launch/trajectory_planning.launch#L19-L20)
### Location: Lines 19–20

```xml
<param name="tracking_node/local_v_max" value="$(arg local_v_max)"/>    <!-- passed from HIT_intergration_test.launch line 12 -->
<param name="tracking_node/local_a_max" value="6.0"/>
```

- **`local_v_max` = `2.8` m/s** (from HIT_intergration_test.launch, can be overridden here)
  - Maximum velocity limit for local planner
  - **Adjust**: Change value in [trajectory_planning.launch](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/launch/trajectory_planning.launch#L19) or pass via launch argument

- **`local_a_max` = `6.0` m/s²** — Maximum acceleration limit
  - Higher = faster acceleration, more aggressive maneuvers
  - Lower = smoother, more cautious acceleration
  - **Adjust**: Change `6.0` to your desired value in [trajectory_planning.launch](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/launch/trajectory_planning.launch#L20)

**Other related limits in same file:**
```xml
<param name="tracking_node/local_w_max" value="8.0"/>      <!-- angular velocity limit (rad/s) -->
<param name="tracking_node/local_j_max" value="8.0"/>      <!-- jerk limit (m/s³) -->
```

---

## 9. **QP Solver Regularization (HPIPM)**

### File: [local_planner.cpp](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/local_planner.cpp#L184-L189)
### Location: Lines 184–189

```cpp
auto& hSet = mpcInterface_.sqpSettings().hpipmSettings;
hSet.hpipmMode   = hpipm_mode::ROBUST;  // was SPEED
hSet.reg_prim    = 1e-8;                 // was 1e-12
```

- **`reg_prim` = `1e-8`** — QP regularization parameter
  - Higher values → more stable solver, slower convergence
  - Lower values → faster but less stable
  - **Adjust**: Change `1e-8` to your desired value (e.g., `1e-6` for more stability)
  - **When to increase**: If you see rare solver failures or QP numerical issues

- **`hpipmMode`** = `ROBUST` — Solver algorithm mode
  - `ROBUST` = more stable, slightly slower
  - `SPEED` = faster but less stable
  - **Adjust**: Change between `ROBUST` or `hpipm_mode::SPEED`

---

## 10. **Obstacle Search Radius**

### File: [local_planner.cpp](HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/local_planner.cpp#L130-L137)
### Location: Lines 130–137

```cpp
for(int i = -10; i<=10; i++){
    for(int j = -10; j<=10; j++){
        Eigen::Vector3i temp_idx = {pos_idx.x() + i, pos_idx.y() + j, pos_idx.z()};
        if(global_map->isOccupied(pos_idx.x() + i, pos_idx.y() + j, pos_idx.z()))
        {
            Eigen::Vector3d temp_pos = global_map->gridIndex2coord(temp_idx);
            obs_point_temp.push_back(temp_pos);
```

- **Search range = `-10` to `+10` grid cells**
  - **At 0.05 m resolution → ±0.5 m**
  - Higher values (e.g., `-15` to `+15`) → searches farther, slower but more aware
  - Lower values (e.g., `-5` to `+5`) → searches closer, faster but may miss distant obstacles
  - **Adjust**: Change `10` to your desired cell count

---

## Summary Table

| Parameter | File | Line(s) | Current Value | Range |
|-----------|------|---------|---------------|-------|
| **mu** (collision) | local_planner.cpp | 179 | 2.0 | 0.5–10 |
| **delta** (smoothness) | local_planner.cpp | 179 | 0.1 | 0.01–0.5 |
| **distance_threshold** | SentryRobotCollisionConstraint.h | 37 | 0.36 m | variable |
| **inequalityConstraintMu** | task.info | 8 | 0.1 | variable |
| **inequalityConstraintDelta** | task.info | 9 | 0.2 | variable |
| **Q diagonal** (states) | task.info | 36–43 | 80, 80, 15, 70 | variable |
| **R diagonal** (inputs) | task.info | 46–49 | 1.5, 0.1 | variable |
| **reference_desire_speed** | HIT_intergration_test.launch | 11 | 2.8 m/s | variable |
| **robot_radius** | HIT_intergration_test.launch | 10 | 0.35 m | variable |
| **local_v_max** | trajectory_planning.launch | 19 | 2.8 m/s | variable |
| **local_a_max** | trajectory_planning.launch | 20 | 6.0 m/s² | variable |
| **reg_prim** | local_planner.cpp | 186 | 1e-8 | 1e-12 to 1e-4 |
| **hpipmMode** | local_planner.cpp | 185 | ROBUST | ROBUST / SPEED |
| **Obstacle search radius** | local_planner.cpp | 130–137 | ±10 cells | ±5 to ±20 cells |

---

## Tuning Workflow

1. **Start conservative** (larger obstacles, slower speeds, high penalties)
2. **Incrementally tune** one parameter at a time
3. **Test in simulation** before real robot deployment
4. **Monitor solver status** in local_planner.cpp output for convergence issues
