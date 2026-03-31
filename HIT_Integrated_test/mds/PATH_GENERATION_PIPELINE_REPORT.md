# Sentry Path Generation Pipeline — Complete Technical Report

> **Generated from source code analysis + live ROS system verification**  
> **Workspace**: `RM_Sentry_2026/HIT_code/sentry_planning_ws/src/sentry_planning/`

---

## Table of Contents

1. [Pipeline Overview (End-to-End Flow)](#1-pipeline-overview)
2. [Stage 1: Goal Input](#2-stage-1-goal-input)
3. [Stage 2: Topological PRM Graph Search](#3-stage-2-topological-prm-graph-search)
4. [Stage 3: A\* Path Pruning (Visibility Shortcutting)](#4-stage-3-a-path-pruning)
5. [Stage 4: MINCO / L-BFGS Joint Space-Time Smoothing](#5-stage-4-minco-l-bfgs-smoothing)
6. [Stage 5: Cubic Spline Polynomial Fitting](#6-stage-5-cubic-spline-polynomial-fitting)
7. [Stage 6: trajectoryPoly ROS Message (Global → Local)](#7-stage-6-trajectorypoly-ros-message)
8. [Stage 7: Local Trajectory Sampling](#8-stage-7-local-trajectory-sampling)
9. [Stage 8: MPC Reference Construction](#9-stage-8-mpc-reference-construction)
10. [Stage 9: OCS2 SQP-MPC Solve](#10-stage-9-ocs2-sqp-mpc-solve)
11. [Stage 10: World→Gimbal Frame + slaver_speed](#11-stage-10-world-gimbal-frame)
12. [Stage 11: hit_bridge → cmd_vel](#12-stage-11-hit-bridge)
13. [Stage 12: mcu_communicator → Serial](#13-stage-12-mcu-communicator-serial)
14. [Complete Topic/Node Map](#14-complete-topic-node-map)
15. [Data Type Reference](#15-data-type-reference)
16. [Parameters Reference](#16-parameters-reference)

---

## 1. Pipeline Overview

```
                        GLOBAL PLANNER                                LOCAL TRACKER                        BRIDGE
                  (trajectory_generation node)                     (tracking_node)                    (hit_bridge + mcu_comm)

 /clicked_point ──► ReplanFSM                                                                        
                     │                                                                               
                     ▼                                                                               
               ┌─────────────┐                                                                       
               │ TopoSearch   │  PRM graph + Dijkstra                                                
               │ createGraph()│──► min_path (sparse 3D waypoints)                                    
               └──────┬──────┘                                                                       
                      ▼                                                                               
               ┌─────────────┐                                                                       
               │ smoothTopo   │  Visibility-based pruning                                            
               │ Path()       │──► optimized_path (pruned waypoints)                                 
               └──────┬──────┘                                                                       
                      ▼                                                                               
               ┌─────────────┐                                                                       
               │ path_smooth  │  L-BFGS + MINCO jerk minimization                                   
               │ smoothPath() │──► final_path (smooth 2D waypoints)                                  
               └──────┬──────┘                                                                       
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │reference_path│  Cubic spline fitting +                                              
               │getRefTraj()  │  trapezoidal time allocation                                         
               │              │──► m_polyMatrix_x/y (Nx4)                                            
               │              │    m_trapezoidal_time (Nx1)                                          
               └──────┬───────┘                                                                      
                      │                                                                               
                      │  /trajectory_generation/global_trajectory                                     
                      │  (trajectoryPoly msg: coef_x, coef_y, duration)                              
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │ LocalPlanner │  Samples polynomial at dt=0.1s                                       
               │rcvGlobalTraj │──► reference_path[] + reference_velocity[]                            
               └──────┬───────┘                                                                      
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │  linearation │  Builds 20-step MPC reference                                        
               │getFightTrack │──► ref_trajectory, ref_velocity,                                     
               │  ingTraj()   │    ref_speed, ref_phi, obs_points                                    
               └──────┬───────┘                                                                      
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │  solveNMPC() │  OCS2 SQP-MPC (unicycle model)                                      
               │              │──► predict_input = [v, φ, a, ω]                                     
               └──────┬───────┘                                                                      
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │tracking_mgr  │  World→gimbal frame rotation                                        
               │rcvLidarIMU   │──► MPC_Control = [gimbal_vx, gimbal_vy, 0, mode]                     
               │  PosCallback │                                                                      
               └──────┬───────┘                                                                      
                      │  /sentry_des_speed  (slaver_speed msg)                                       
                      ▼                                                                               
               ┌──────────────┐                                                                      
               │  hit_bridge  │  Passthrough slaver_speed → Twist                                    
               │              │──► twist.linear.x = vx, twist.linear.y = vy                          
               └──────┬───────┘                                                                      
                      │  /cmd_vel  (geometry_msgs/Twist)                                             
                      ▼                                                                               
               ┌──────────────────┐                                                                  
               │ mcu_communicator │  Serial USB at 50Hz                                              
               │                  │──► NavigationFrame{vx, vy} → MCU                                
               └──────────────────┘                                                                  
```

---

## 2. Stage 1: Goal Input

### Source File
`trajectory_generation/src/replan_fsm.cpp` — `ReplanFSM::rcvTargetCallback()`

### What Happens
The global planner receives a goal position via `/clicked_point` (type `geometry_msgs/PointStamped`). In the real system, this is published by `local_frame_bridge` which converts decision-layer goals from the local frame to the map frame.

### Key Code
```cpp
// replan_fsm.cpp line 63
target_point_sub = nh.subscribe("/clicked_point", 1, &ReplanFSM::rcvTargetCallback, this);
```

The FSM transitions: `INIT → WAIT_TARGET → GEN_NEW_TRAJ`.

### ROS Topic
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/clicked_point` | `geometry_msgs/PointStamped` | `local_frame_bridge` | `trajectory_generation` |

---

## 3. Stage 2: Topological PRM Graph Search

### Source Files
- `trajectory_generation/src/TopoSearch.cpp` — `TopoSearcher::createGraph()`
- `trajectory_generation/src/plan_manager.cpp` — `planner_manager::pathFinding()` (orchestrator)

### What Happens

1. **Graph Construction** (`createGraph()`):
   - Seeds **guard nodes** from a pre-computed skeleton image (`occtopo.png`) — keypoints on the Voronoi skeleton of free space.
   - Samples up to **500 random points** inside an ellipsoidal region between start and goal.
   - Classifies each sample as a **guard** (far from existing guards) or **connector** (visible to ≥2 guards).
   - Builds an adjacency graph with edge costs = Euclidean distance.

2. **Dijkstra Search** (`DijkstraSearch()`):
   - Runs single-source shortest-path from start guard to goal guard on the PRM graph.
   - Output: `min_path` — a `std::vector<Eigen::Vector3d>` of **sparse** 3D waypoints (typically 5–20 points depending on map complexity).

3. **Height Awareness**:
   - Uses `heightFeasible()` to verify each edge is traversable (checks height gradient for stairs/ramps via `RM_GridMap`).

### Key Code
```cpp
// plan_manager.cpp line 270
topo_prm->createGraph(start_pt, target_pt);
// ...
origin_path = topo_prm->min_path;  // sparse waypoints from Dijkstra
```

### Data Flow
```
Input:  start_pt (Vector3d), target_pt (Vector3d), occupancy map
Output: min_path = vector<Vector3d>  (sparse topological waypoints)
```

---

## 4. Stage 3: A\* Path Pruning (Visibility Shortcutting)

### Source File
`trajectory_generation/src/Astar_searcher.cpp` — `AstarPathFinder::smoothTopoPath()`

### What Happens
Takes the sparse Dijkstra path and prunes unnecessary waypoints via **visibility checks**:
- From each point, looks ahead as far as possible with line-of-sight (ray-casting on the grid map).
- Skips intermediate points that are occluded by obstacles.
- Result: a **shorter** path with fewer, more spread-out waypoints.

### Key Code
```cpp
// plan_manager.cpp line 287
optimized_path = astar_path_finder->smoothTopoPath(origin_path);
```

### Data Flow
```
Input:  origin_path (vector<Vector3d>, ~5-20 pts)
Output: optimized_path (vector<Vector3d>, ≤ input size, usually fewer)
```

---

## 5. Stage 4: MINCO / L-BFGS Joint Space-Time Smoothing

### Source File
`trajectory_generation/src/path_smooth.cpp` — `Smoother::smoothPath()`

### What Happens
The pruned waypoints are passed through a **joint space-time optimization** using L-BFGS with MINCO (Minimum Control) theory:

1. **Initialization** (`Smoother::init()`):
   - Resamples the pruned path at **0.3m intervals**.
   - Sets up decision variable vector: `[x₁..xₙ₋₁, y₁..yₙ₋₁, τ₁..τₙ]`
     - Inner waypoint positions (start/end fixed)
     - Unconstrained time variables via diffeomorphism: `τᵢ = ln(Tᵢ)`, so `Tᵢ = eᵗⁱ > 0` always.

2. **L-BFGS Optimization** (`smoothPath()`):
   - **Max 200 iterations**
   - **Cost function** is a weighted sum:

     | Cost Term | Weight | Purpose |
     |-----------|--------|---------|
     | `wSmooth` = 2e-2 | Jerk energy from MINCO | Smoothness |
     | `wFidelity` = 20 | Elastic band to original path | Stay near reference |
     | `wVel` = 1e3 | Velocity limit penalty | Feasibility |
     | `wMinTime` = 1e3 | Minimum total time | Efficiency |
     | `wObstacle` = 1e5 | Obstacle proximity cost | Safety |

   - **Drift clamp**: waypoints cannot drift more than **0.8m** from their reference positions.

3. **Resample** (`pathResample()`):
   - Decimates to every 2nd point to reduce resolution for the cubic spline stage.

4. **Output** (`getPath()`):
   - `final_path`: `vector<Eigen::Vector2d>` — smooth, evenly-spaced 2D waypoints.
   - `m_trapezoidal_time`: Optimized segment durations (from the time diffeomorphism).

### Key Code
```cpp
// plan_manager.cpp lines 296-300
path_smoother->init(optimized_path, start_vel, reference_speed);
path_smoother->smoothPath();
path_smoother->pathResample();
final_path = path_smoother->getPath();
```

### MINCO Theory Note
The `minco_trajectory.hpp` defines a **quintic (5th-order)** piecewise polynomial for jerk minimization — the MINCO formulation parametrizes trajectories by inner waypoints and durations, then computes the minimum-jerk trajectory through those waypoints analytically. The L-BFGS optimizer moves both waypoints and durations to minimize the combined cost.

### Data Flow
```
Input:  optimized_path (vector<Vector3d>), start_vel, reference_speed
Output: final_path    (vector<Vector2d>, smooth waypoints, ~30-100 pts)
        m_trapezoidal_time (vector<double>, optimized segment durations)
```

---

## 6. Stage 5: Cubic Spline Polynomial Fitting

### Source File
`trajectory_generation/src/reference_path.cpp` — `Refenecesmooth::getRefTrajectory()`

### What Happens
The smooth waypoints are fitted with **cubic splines** (3rd-order piecewise polynomials) to produce the final publishable trajectory:

1. **Time Allocation** (`solveTrapezoidalTime()`):
   - If MINCO-optimized times are available, use those directly.
   - Otherwise: `time_i = distance_i / desire_velocity`, with adjustments:
     - `bridge_coeff`: slower through bridge/tunnel areas (detected by `exist_second_height`)
     - `slope_coeff`: slower on slopes proportional to gradient

2. **Banded Linear System** (`solvePolyMatrix()`):
   - Sets up a **tridiagonal (banded) system** for the cubic spline second derivatives:
     ```
     [2T₀   T₀    0   ...] [M₀]   [6·(Δx₁/T₁ - v₀)]
     [T₀  2(T₀+T₁) T₁ ...] [M₁] = [6·(Δxᵢ₊₁/Tᵢ₊₁ - Δxᵢ/Tᵢ)]
     [... ... ... ...      ] [..]   [...]
     ```
   - Solves via LU factorization of the banded matrix.
   - Computes polynomial coefficients per segment:
     ```
     x(t) = d·t³ + c·t² + b·t + a
     
     d = (M_{i+1} - M_i) / (6·T_i)
     c = M_i / 2
     b = (x_{i+1} - x_i)/T_i - T_i·(2·M_i + M_{i+1})/6
     a = x_i
     ```
   - Boundary condition: Initial velocity from robot state; terminal velocity = 0 (natural spline).

3. **Feasibility Check** (`checkfeasible()`):
   - Evaluates the polynomial velocity profiles.
   - If acceleration exceeds `max_acceleration`, inflates the corresponding segment duration and re-solves (up to 3 iterations).

### Key Code
```cpp
// reference_path.cpp lines 193-202  (coefficient assignment)
m_polyMatrix_x(i, 0) = (b(i+1, 0) - b(i, 0)) / (6 * m_trapezoidal_time[i]);  // d (t³ coeff)
m_polyMatrix_x(i, 1) = b(i, 0) / 2;                                            // c (t² coeff)
m_polyMatrix_x(i, 2) = (path[i+1].x - path[i].x)/T - T*(2*b(i)+b(i+1))/6;    // b (t¹ coeff)
m_polyMatrix_x(i, 3) = m_global_path[i].x();                                    // a (constant)
```

### Output Data Structure
For N segments:
- `m_polyMatrix_x`: Eigen::MatrixXd(N, 4) — columns are [d, c, b, a] for x-axis
- `m_polyMatrix_y`: Eigen::MatrixXd(N, 4) — columns are [d, c, b, a] for y-axis
- `m_trapezoidal_time`: vector<double>(N) — duration of each segment in seconds

### Data Flow
```
Input:  final_path (vector<Vector2d>), start_vel, desire_speed, m_trapezoidal_time (from MINCO)
Output: m_polyMatrix_x  (Nx4, cubic polynomial coefficients for x)
        m_polyMatrix_y  (Nx4, cubic polynomial coefficients for y)
        m_trapezoidal_time (Nx1, segment durations in seconds)
```

---

## 7. Stage 6: trajectoryPoly ROS Message (Global → Local)

### Source File
`trajectory_generation/src/replan_fsm.cpp` — `ReplanFSM::execFSMCallback()`

### What Happens
The polynomial coefficients and durations are serialized into a custom ROS message and published:

### Key Code
```cpp
// replan_fsm.cpp lines 318-339
trajectory_generation::trajectoryPoly global_path;
global_path.motion_mode = decision_mode;
for(int i = 0; i < plannerManager->reference_path->m_trapezoidal_time.size(); i++){
    global_path.duration.push_back(plannerManager->reference_path->m_trapezoidal_time[i]);
    global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 0)); // d
    global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 1)); // c
    global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 2)); // b
    global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 3)); // a
    // ...same for coef_y...
}
global_planning_result_pub.publish(global_path);
```

### Message Layout
For N segments, the flat arrays contain:
```
coef_x = [d₀, c₀, b₀, a₀,  d₁, c₁, b₁, a₁,  ...,  dₙ₋₁, cₙ₋₁, bₙ₋₁, aₙ₋₁]
          ├── segment 0 ───┤  ├── segment 1 ───┤       ├── segment N-1 ────────┤
          (4 floats/seg)      (4 floats/seg)            (4 floats/seg)

duration = [T₀, T₁, ..., Tₙ₋₁]   (seconds per segment)
```

Polynomial evaluation for segment i at local time t ∈ [0, Tᵢ]:
```
x(t) = d·t³ + c·t² + b·t + a
     = coef_x[4i+0]·t³ + coef_x[4i+1]·t² + coef_x[4i+2]·t + coef_x[4i+3]
```

### ROS Topic
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/trajectory_generation/global_trajectory` | `trajectory_generation/trajectoryPoly` | `trajectory_generation` | `tracking_node` |

---

## 8. Stage 7: Local Trajectory Sampling

### Source File
`trajectory_tracking/src/local_planner.cpp` — `LocalPlanner::rcvGlobalTrajectory()`

### What Happens
The local planner receives the polynomial coefficients and reconstructs a dense, uniformly-sampled reference path and velocity profile.

1. **Coefficient Unpacking** (lines 632–649):
   ```cpp
   m_polyMatrix_x(i, 0) = polytraj->coef_x[4*i + 0];  // d
   m_polyMatrix_x(i, 1) = polytraj->coef_x[4*i + 1];  // c
   m_polyMatrix_x(i, 2) = polytraj->coef_x[4*i + 2];  // b
   m_polyMatrix_x(i, 3) = polytraj->coef_x[4*i + 3];  // a
   ```
   (NaN/Inf checks on all coefficients; trajectory discarded if corrupt.)

2. **Position Sampling** (`getRefTrajectory()`, line 1118):
   - Evaluates the polynomial at **dt = 0.1s** intervals:
     ```cpp
     ref_point.x = d·(t-t₀)³ + c·(t-t₀)² + b·(t-t₀) + a
     ```
   - Produces `reference_path`: `vector<Vector3d>` — one point every 0.1s.

3. **Velocity Sampling** (`getRefVel()`, line 1142):
   - Evaluates the polynomial **derivative** at dt = 0.1s:
     ```cpp
     ref_vel.x = 3·d·(t-t₀)² + 2·c·(t-t₀) + b
     ```
   - Produces `reference_velocity`: `vector<Vector3d>` — one velocity every 0.1s.

4. **State Reset**: Tracking time reset, MPC solver cold-started (`mpcSolverPtr_->reset()`), motion_mode captured.

### Data Flow
```
Input:  trajectoryPoly msg (coef_x, coef_y, duration, motion_mode)
Output: reference_path     (vector<Vector3d>, sampled every 0.1s)
        reference_velocity (vector<Vector3d>, sampled every 0.1s)
        duration_time      (vector<double>, segment durations)
```

---

## 9. Stage 8: MPC Reference Construction

### Source File
`trajectory_tracking/src/local_planner.cpp` — `LocalPlanner::getFightTrackingTraj()` → `linearation()`

### What Happens
Every control cycle (~10Hz, triggered by `/odom_local` callback), the tracking manager calls this to build the MPC's 20-step lookahead reference:

#### 9a. `getFightTrackingTraj()` (line 287)

1. **Find closest point** on `reference_path` to robot's current position.
2. **Lead-time capping**: MAX_LEAD_TIME = **0.5s** — don't let the reference race ahead of the robot.
3. Calls `linearation()` to build the horizon reference.
4. **Heading computation**: For each of the 20 steps, computes `ref_phi[i]` from `atan2(ref_vy, ref_vx)` with:
   - **Reverse mode** support: `speed_direction` flips to −1 when heading differs >108° from robot yaw (hysteresis at 63° to exit).
   - Phase unwrapping across 2π boundaries.
5. **Deceleration ramp**: Within 1.5m of goal, `ref_speed[i] *= max(0.05, dist/1.5)`.
6. **Obstacle-proximity speed reduction**: Two-pass min(step_scale, horizon_scale) with 1.5m brake distance and 0.25 minimum scale.

#### 9b. `linearation()` (line 17)

1. **Time → segment index**: Finds which polynomial segment corresponds to current tracking time.
2. **Builds 20-step MPC references** by **linear interpolation** between sampled reference points:
   ```cpp
   position_temp.x = reference_path[seg].x 
       + (reference_path[seg+1].x - reference_path[seg].x) * segment_time/dt;
   ```
3. **Obstacle search** for each of 20 horizon steps (±20 cells, ~1.0m radius):
   - Searches around **reference trajectory** point
   - Searches around **previous predicted trajectory** (for diverged paths)
   - Searches around **robot position** (first 5 steps)
   - **Deduplication** + **8-sector angular selection** (closest obstacle per 45° sector)
   - Max 1.0m distance filter
   - Output: `obs_points[20]` — up to 8 obstacle positions per MPC step.

4. **OCS2 obstacle constraint update**: Writes obstacle data directly to the solver's shared constraint object:
   ```cpp
   mpcInterface_.obsConstraintPtr_->obs_points_t_ = obs_points_t;
   ```

### Data Flow
```
Input:  robot state (x, y, v, φ), current time
Output: ref_trajectory[20]  (Vector3d, position per MPC step)
        ref_velocity[20]    (Vector3d, world-frame velocity)
        ref_speed[20]       (double, scalar speed with decel + obs scaling)
        ref_phi[20]         (double, heading angle)
        obs_points[20]      (vector<Vector3d>, obstacle positions per step)
        reference_time[20]  (double, absolute timestamps)
```

---

## 10. Stage 9: OCS2 SQP-MPC Solve

### Source Files
- `trajectory_tracking/src/local_planner.cpp` — `LocalPlanner::solveNMPC()`
- `trajectory_tracking/src/ocs2_sentry/SentryRobotInterface.cpp` — MPC parameters
- `trajectory_tracking/src/ocs2_sentry/SentryRobotCollisionConstraint.cpp` — Collision barrier

### What Happens

1. **Reference assembly** (`solveNMPC()`, line 801):
   - Packs reference into OCS2 `TargetTrajectories`:
     ```cpp
     reference_state = {ref_trajectory[i].x, ref_trajectory[i].y, ref_speed[i], ref_phi[i]};
     ```
   - State vector: `[x, y, v, φ]` — position, speed, heading.
   - Input vector: `[a, ω]` — acceleration, angular velocity (zero reference).

2. **Observation** (`solveNMPC()`, line 846):
   - `observation.state = [robot_x, robot_y, line_speed, velocity_heading]`
   - `observation.time = tracking_time`

3. **SQP Solve** (`mpcSolverPtr_->run()`):
   - **Model**: Unicycle kinematics:
     ```
     ẋ = v·cos(φ)
     ẏ = v·sin(φ)
     v̇ = a   (input)
     φ̇ = ω   (input)
     ```
   - **Horizon**: 20 steps × 0.1s = 2.0s
   - **Tracking cost** (quadratic): `Q = diag(80, 80, 15, 70)`, `R = diag(1.5, 0.1)`
   - **Collision avoidance**: Relaxed barrier penalty on obstacle distances
     - μ = 20, δ = 0.5
     - Static threshold: 0.04 m² (≈0.2m)
     - Dynamic threshold: 0.16 m² (≈0.4m)
   - **HPIPM settings**: ROBUST mode, reg_prim = 1e-8
   - **State/input bounds**: Relaxed barrier on [v_min, v_max], [a_min, a_max], [ω_min, ω_max]

4. **Output extraction** (`getNMPCPredictXU()`, line 923):
   ```cpp
   predict_state(0) = stateTrajectory_[1](2);  // v at next step
   predict_state(1) = stateTrajectory_[1](3);  // φ at next step
   predict_state(2) = inputTrajectory_[0](0);  // a (acceleration)
   predict_state(3) = inputTrajectory_[0](1);  // ω (angular velocity)
   ```
   - **Key**: The MPC output is `[v, φ, a, ω]` — a **world-frame** velocity and heading.

### Data Flow
```
Input:  sentry_state = [x, y, v, φ]  (robot observation)
        targetTrajectories  (20-step reference: [x,y,v,φ] per step)
        obs_points_t  (obstacles w/ type tags per step)

Output: predict_input = [v, φ, a, ω]  (first MPC control action)
        predictState[20]  (full predicted state trajectory for viz + feasibility)
```

---

## 11. Stage 10: World→Gimbal Frame + slaver_speed

### Source File
`trajectory_tracking/src/tracking_manager.cpp` — `rcvLidarIMUPosCallback()` + `publishSentryOptimalSpeed()`

### What Happens

1. **World-frame velocity** from MPC output:
   ```cpp
   vx_world = v_ctrl * cos(phi_ctrl);
   vy_world = v_ctrl * sin(phi_ctrl);
   ```

2. **World → Gimbal-frame rotation** using LiDAR yaw:
   ```cpp
   // R^T * v_world  (inverse rotation)
   MPC_Control(0) =  vx_world * cos(yaw) + vy_world * sin(yaw);  // gimbal vx (forward)
   MPC_Control(1) = -vx_world * sin(yaw) + vy_world * cos(yaw);  // gimbal vy (left)
   ```
   (Note: Despite the field names `angle_target`/`angle_current`, these now carry **gimbal-frame linear velocities**, not angles.)

3. **Packaging** (`publishSentryOptimalSpeed()`, line 864):
   ```cpp
   sentry_speed.angle_target  = speed(0);  // gimbal-frame vx
   sentry_speed.angle_current = speed(1);  // gimbal-frame vy
   sentry_speed.line_speed    = speed(2);  // unused (0)
   sentry_speed.xtl_flag      = mode;      // 0=attack, 1=defense, 2=move, 3=brake
   sentry_speed.in_bridge     = in_bridge; // true if under bridge
   ```

4. **Motion mode voting**: Maintains a 40-sample sliding window of mode values; publishes the majority vote with 38/40 confirmation threshold.

### ROS Topic
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/sentry_des_speed` | `sentry_msgs/slaver_speed` | `tracking_node` | `hit_bridge` |

### Data Flow
```
Input:  predict_input = [v, φ, a, ω], robot_lidar_yaw
Output: slaver_speed msg {
          angle_target  = gimbal_vx,  (m/s)
          angle_current = gimbal_vy,  (m/s)
          line_speed    = 0,
          xtl_flag      = motion_mode (0-3),
          in_bridge     = bool
        }
```

---

## 12. Stage 11: hit_bridge → cmd_vel

### Source File
`trajectory_tracking/src/hit_bridge.cpp`

### What Happens
Simple passthrough node — converts `slaver_speed` to `geometry_msgs/Twist`:

```cpp
void speedCallback(const sentry_msgs::slaver_speed::ConstPtr& msg)
{
    geometry_msgs::Twist twist;
    twist.linear.x  = msg->angle_target;   // world-frame vx passthrough
    twist.linear.y  = msg->angle_current;  // world-frame vy passthrough
    twist.linear.z  = 0.0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
    twist_pub.publish(twist);
    
    // Also publishes arrival flag
    std_msgs::Bool arrived;
    arrived.data = (fabs(msg->angle_target) < 0.01 && fabs(msg->angle_current) < 0.01);
    arrived_pub.publish(arrived);
}
```

### ROS Topics
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | `hit_bridge` | `mcu_communicator` |
| `/dstar_status` | `std_msgs/Bool` | `hit_bridge` | `mcu_communicator` |

---

## 13. Stage 12: mcu_communicator → Serial

### Source Files
- `DecisionNode/src/decision_node/src/mcu_communicator.cpp`
- `DecisionNode/src/decision_node/include/decision_node/mcu_comm.hpp`

### What Happens

1. **Subscription**:
   ```cpp
   void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
   {
       current_nav_vx_ = msg->linear.x;
       current_nav_vy_ = msg->linear.y;
   }
   ```

2. **Timer-based serial output at 50Hz** (`navigationTimerCallback()`):
   ```cpp
   void sendNavigationCommand(float vx, float vy)
   {
       NavigationFrame frame;
       frame.vx = vx;
       frame.vy = vy;
       serial_.write((uint8_t*)&frame, sizeof(frame));
   }
   ```

3. **Serial frame format** (current, `mcu_comm.hpp`):
   ```c
   struct NavigationFrame {
       float vx;   // world-frame vx [m/s] → MCU vision_rx.v
       float vy;   // world-frame vy [m/s] → MCU vision_rx.w
   } __attribute__((packed));
   // Total: 8 bytes, raw floats, no SOF/EOF/CRC (USB CDC preserves boundaries)
   ```

4. **MCU side**: The MCU receives `vision_rx_t { float v; float w; }` and drives the chassis motors to achieve the commanded vx/vy in the gimbal frame.

### Full I/O
| Direction | Data | Format |
|-----------|------|--------|
| NUC → MCU | NavigationFrame(vx, vy) | 8 bytes raw float × 2 @ 50Hz |
| MCU → NUC | MCUDataFrame(yaw_angle) | 4 bytes raw float (gimbal yaw in degrees) |

---

## 14. Complete Topic/Node Map

### Nodes (verified on running system)
| Node | Package | Purpose |
|------|---------|---------|
| `/trajectory_generation` | trajectory_generation | Global planner: FSM + path planning |
| `/tracking_node` | trajectory_tracking | Local tracker: MPC + control |
| `/hit_bridge` | trajectory_tracking | slaver_speed → Twist conversion |
| `/mcu_communicator` | decision_node | Twist → serial USB bridge |
| `/local_frame_bridge` | decision_node | Local→map frame converter for goals + odom |

### All Planning Topics (verified with `rostopic info`)

| Topic | Type | Publisher(s) | Subscriber(s) | Description |
|-------|------|-------------|---------------|-------------|
| `/clicked_point` | `geometry_msgs/PointStamped` | `local_frame_bridge` | `trajectory_generation` | Goal position in map frame |
| `/odom_local` | `nav_msgs/Odometry` | `local_frame_bridge` | `trajectory_generation`, `tracking_node` | Robot odometry (position + body-frame twist) |
| `/trajectory_generation/global_trajectory` | `trajectory_generation/trajectoryPoly` | `trajectory_generation` | `tracking_node` | Cubic polynomial trajectory + time allocation |
| `/replan_flag` | `std_msgs/Bool` | `tracking_node` | `trajectory_generation` | Request global replanning when MPC detects collision |
| `/sentry_des_speed` | `sentry_msgs/slaver_speed` | `tracking_node` | `hit_bridge` | MPC output: gimbal-frame vx/vy + mode |
| `/cmd_vel` | `geometry_msgs/Twist` | `hit_bridge` | `mcu_communicator` | Final velocity command |
| `/dstar_status` | `std_msgs/Bool` | `hit_bridge` | `mcu_communicator` | Arrival flag (v ≈ 0) |
| `/slaver/wheel_state` | `std_msgs/Bool` | *(external)* | `trajectory_generation` | Wheel encoder health check |
| `/solver_status` | `std_msgs/Bool` | `tracking_node` | — | MPC solver success/fail diagnostic |
| `/redecide_flag` | `std_msgs/Bool` | `tracking_node` | *(decision layer)* | Request decision re-evaluation |

### Feedback Loop

The system has a **closed-loop replanning architecture**:
```
tracking_node ──/replan_flag──► trajectory_generation
                                       │
                                       ▼
                              (re-runs pathFinding
                               from current position)
                                       │
                              /global_trajectory
                                       │
                                       ▼
                              tracking_node (new trajectory received,
                                            cold-starts MPC)
```

Replan trigger conditions (from `tracking_manager.cpp`):
1. **Collision detected**: MPC predicted path hits dynamic obstacle for > 67% of check window
2. **Off-course**: Robot > 0.8m from nearest reference point for > 40% of check window
3. **Cooldown**: 0.5s minimum between replans to prevent churn

---

## 15. Data Type Reference

### `trajectory_generation/trajectoryPoly.msg`
```
time     start_time
uint8    motion_mode       # 0=attack, 1=defense, 2=move
float32[] coef_x           # 4 floats per segment: [d, c, b, a]
float32[] coef_y           # 4 floats per segment: [d, c, b, a]
float32[] duration         # 1 float per segment: Tᵢ (seconds)
```

### `sentry_msgs/slaver_speed.msg`
```
float32 line_speed       # unused (0)
float32 angle_target     # gimbal-frame vx (m/s)
float32 angle_current    # gimbal-frame vy (m/s)
uint8   xtl_flag         # motion mode: 0=attack, 1=defense, 2=move, 3=brake
uint8   in_bridge        # 1 if under bridge/tunnel
```

### `geometry_msgs/Twist` (as used on `/cmd_vel`)
```
linear.x  = gimbal-frame vx (m/s)
linear.y  = gimbal-frame vy (m/s)
linear.z  = 0
angular.* = 0
```

### Serial: `NavigationFrame` (NUC → MCU)
```c
struct NavigationFrame {
    float vx;   // gimbal-frame vx [m/s], maps to MCU vision_rx.v
    float vy;   // gimbal-frame vy [m/s], maps to MCU vision_rx.w
};  // 8 bytes, unframed (USB CDC)
```

---

## 16. Parameters Reference

### Global Planner (`trajectory_generation`)
| Parameter | Default | Source | Description |
|-----------|---------|--------|-------------|
| `reference_desire_speed` | 1.6 | launch | Cruise speed for time allocation |
| `reference_a_max` | 6.0 | launch | Max accel for feasibility check |
| `slope_coeff` | *(config)* | ref_path | Time inflation on slopes |
| `bridge_coeff` | *(config)* | ref_path | Time inflation through bridges |
| `wSmooth` | 2e-2 | path_smooth | L-BFGS jerk weight |
| `wFidelity` | 20 | path_smooth | L-BFGS elastic band weight |
| `wObstacle` | 1e5 | path_smooth | L-BFGS obstacle weight |

### Local Tracker (`tracking_node`)
| Parameter | Default | Source | Description |
|-----------|---------|--------|-------------|
| `dt` | 0.1 | launch | MPC time step (seconds) |
| `planning_horizon` | 20 | launch | MPC horizon length |
| `local_v_max` | 6.0 | launch | Max velocity |
| `local_a_max` | 6.0 | launch | Max acceleration |
| `local_w_max` | 8.0 | launch | Max angular velocity |
| Q matrix | diag(80,80,15,70) | task.info | State tracking cost |
| R matrix | diag(1.5, 0.1) | task.info | Input regularization |
| Barrier μ/δ | 20/0.5 | local_planner.cpp | Collision barrier strength |
| MAX_LEAD_TIME | 0.5s | local_planner.cpp | Reference ahead limit |
| OFF_COURSE_DIST | 0.8m | local_planner.cpp | Replan trigger threshold |
| OBS_SLOW_DIST | 1.5m | local_planner.cpp | Obstacle braking distance |

---

## Summary: The Complete Data Transform Chain

```
Goal (x,y)
    │
    ▼
TopoSearch::createGraph()          → min_path: vector<Vector3d>         [sparse PRM waypoints]
    │
    ▼
AstarPathFinder::smoothTopoPath() → optimized_path: vector<Vector3d>   [visibility-pruned]
    │
    ▼
Smoother::smoothPath()             → final_path: vector<Vector2d>       [L-BFGS + MINCO smoothed]
Smoother::pathResample()              m_trapezoidal_time: vector<double> [optimized durations]
    │
    ▼
Refenecesmooth::solvePolyMatrix()  → m_polyMatrix_x: Nx4 [d,c,b,a]     [cubic spline coefficients]
                                     m_polyMatrix_y: Nx4 [d,c,b,a]
                                     m_trapezoidal_time: Nx1             [final segment durations]
    │
    │  ROS: /trajectory_generation/global_trajectory (trajectoryPoly)
    ▼
LocalPlanner::getRefTrajectory()   → reference_path: vector<Vector3d>   [sampled every 0.1s]
LocalPlanner::getRefVel()          → reference_velocity: vector<Vector3d>
    │
    ▼
LocalPlanner::linearation()        → ref_trajectory[20]: Vector3d       [20-step MPC positions]
                                     ref_velocity[20]: Vector3d          [20-step MPC velocities]
                                     ref_speed[20]: double               [scalar speed: decel+obs scaled]
                                     ref_phi[20]: double                 [heading angles, unwrapped]
                                     obs_points[20]: vector<Vector3d>    [obstacles per step, 8 sectors]
    │
    ▼
LocalPlanner::solveNMPC()          → predict_input: [v, φ, a, ω]       [MPC first control action]
    │
    ▼
tracking_manager                   → MPC_Control: [gimbal_vx, gimbal_vy, 0, mode]
  (world→gimbal rotation)                                               [body-frame velocity]
    │
    │  ROS: /sentry_des_speed (slaver_speed)
    ▼
hit_bridge                         → Twist{vx, vy}                      [passthrough]
    │
    │  ROS: /cmd_vel (Twist)
    ▼
mcu_communicator                   → NavigationFrame{vx, vy}           [8 bytes raw float, USB 50Hz]
    │
    ▼
MCU → chassis motors
```
