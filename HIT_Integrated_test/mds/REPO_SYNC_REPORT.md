# RM_Sentry_2026 Repository Sync Report

> **Date:** March 20, 2026  
> **Source:** `/home/sentry_train_test/AstarTraining/` (working development tree)  
> **Target:** `/home/sentry_train_test/AstarTraining/RM_Sentry_2026/` (git repo)  
> **Previous repo state:** commit `b17a77f` — "backup pre-minco code"

---

## Summary

Synchronized the RM_Sentry_2026 git repository with the current working HIT integrated sentry navigation pipeline. This update brings **43 bug fixes**, the **MINCO trajectory optimizer**, the full **DecisionNode package**, all **documentation**, and removes obsolete files.

**Total changes:** 112 files — 39 added, 46 modified, 26 deleted, 1 renamed  
**Net code change:** +11,761 / -398 lines

---

## 1. What Was Synced

### 1.1 Sentry Planning Source Code (All 43 Fixes)

The entire `sentry_planning/` tree was synced from the working directory, incorporating **43 fixes** applied over March 2026. These fixes address:

- **Obstacle visibility** (Fixes 1-4): Temporal decay for `l_data`, refresh bug, height filter tuning, proximity gate replacing binary safe_constraint
- **MPC solver stability** (Fixes 5-7, 42): HPIPM ROBUST mode, solver one-time init, NaN guards, barrier parameter tuning (μ=20, δ=1.0)
- **Collision threshold** (Fix 8): Reduced from 0.36→0.09 (distance²), total clearance 0.65m from actual walls
- **Cost matrix tuning** (Fix 9): Q={150,150,60,80}, R={1.5,0.15}
- **Speed limits** (Fix 11): v_max=2.0m/s, a_max=3.5m/s²
- **Topo graph dynamic obstacles** (Fix 12): `isStaticOccupied()` → `isOccupied()`
- **Path commitment / anti-oscillation** (Fixes 13-14): Skip safe replans, hysteresis, re-anchoring
- **MINCO trajectory optimization** (Fixes 24-29): Quintic (5th order) min-jerk optimizer, L-BFGS with gradient clipping
- **Replan FSM stability** (Fixes 30-38): Cooldown, startup protection, frame alignment
- **THE SMOKING GUN (Fix 43)**: MAX_LEAD_TIME 2.0→0.5s, robot-centered obstacle search for MPC steps 0-4

**Files modified (trajectory_generation):**
| File | Changes |
|------|---------|
| `include/path_smooth.h` | MINCO members, refWaypoints |
| `include/replan_fsm.h` | Replan cooldown, state tracking |
| `include/root_solver/minco_trajectory.hpp` | **NEW** — 685-line quintic MINCO class |
| `launch/global_searcher.launch` | Map params from YAML, speed limits |
| `src/Astar_searcher.cpp` | Obstacle detection improvements |
| `src/RM_GridMap.cpp` | Temporal decay (10 frames), l_data refresh fix |
| `src/TopoSearch.cpp` | Dynamic obstacle visibility in graph |
| `src/path_smooth.cpp` | MINCO L-BFGS optimizer with 5 cost terms |
| `src/plan_manager.cpp` | Path commitment, hysteresis, re-anchoring |
| `src/reference_path.cpp` | External MINCO duration acceptance |
| `src/replan_fsm.cpp` | v2 FSM: cooldown, startup warmup |
| `src/visualization_utils.cpp` | Updated visualization |
| `map/bevfinal.png` | Updated to current arena map |
| `map/map_meta.yaml` | Current arena metadata |
| `map/occfinal.png` | Updated occupancy map |
| `map/occtopo.png` | Updated topological map |

**Files modified (trajectory_tracking):**
| File | Changes |
|------|---------|
| `cfg/task.info` | Q/R weights, SQP params, HPIPM ROBUST |
| `include/local_planner.h` | New members for Fix 43 |
| `include/ocs2_sentry/constraint/SentryRobotCollisionConstraint.h` | Threshold 0.09 (static), 0.49 (dynamic) |
| `include/ocs2_sentry/constraint/SentryRobotStateInputConstraint.h` | v_max=2.5, a_max=3.5, ω_max=6.0 |
| `launch/trajectory_planning.launch` | Speed limits, map params |
| `src/RM_GridMap.cpp` | Temporal decay (5 frames), l_data refresh fix |
| `src/local_planner.cpp` | Fix 43: lead time, 3-origin obstacle search, decel ramp, diagnostics |
| `src/ocs2_sentry/SentryRobotInterface.cpp` | Barrier tuning (0.5, 0.1) |
| `src/ocs2_sentry/constraint/SentryRobotCollisionConstraint.cpp` | Dual threshold, no static extra penalty |
| `src/tracking_manager.cpp` | Replan triggers, collision frame tracking |

### 1.2 HIT_Integrated_test Config & Launch

| File | Action | Description |
|------|--------|-------------|
| `HIT_intergration_test.launch` | Modified | Updated with map_meta.yaml auto-params, mcu_communicator control, BEV/OCC/TOPO viz nodes, use_omega_output flag |
| `hit_env.bash` | Modified | **Now uses relative paths** (`$(dirname)` pattern) — self-contained within repo |
| `test.rviz` | Modified | Updated RViz configuration |
| `HIT_pointlio_map_builder.launch` | Added | Point-LIO map building launch |
| `test_localization_constraints.sh` | Added | Localization testing script |
| `MAP_GRADIENT_FIX_REPORT.md` | Added | Point-LIO gravity calibration report |
| `RVIZ_LAUNCH_README.md` | Updated | RViz/VNC setup guide |

### 1.3 DecisionNode Package (NEW)

Added the complete `DecisionNode/` catkin package — previously only a standalone `mcu_communicator.cpp` existed in HIT_code. The package provides:

- **mcu_communicator** node: Serial bridge (0x93 NavigationFrame, CRC8, 115200 baud)
- **strategy_node**: Decision-making with behavior trees
- **central_occupiable**: Occupancy-based decision support
- **motion_change / recover_change**: Movement recovery behaviors

The old standalone `HIT_code/mcu_communicator.cpp` was renamed/replaced → `DecisionNode/src/decision_node/src/mcu_communicator.cpp`.

### 1.4 Sim_nav Packages

| Package | Files Changed | Key Changes |
|---------|--------------|-------------|
| `bot_sim` | 5 files | imu_filter.launch, map_server*.launch, imu_filter.cpp |
| `hdl_localization` | 3 files | NDT params, launch config, pose_estimator improvements |
| `Point-LIO` | 3 files | avia.yaml, laserMapping.cpp, parameters*.h/cpp |
| `ws_livox` | 1 file | CMakeLists.txt |
| `tools/` | 4 files | tmp_maps updated (bev.png, occ.png, occtopo.png, map_meta.yaml) |

### 1.5 Documentation (10 files added to `mds/`)

| Document | Description |
|----------|-------------|
| `PLANNING_FIX_HISTORY.md` | **1992 lines** — Complete log of all 43 fixes with root causes, code changes, and verification results |
| `PLAN_1_MINCO_SPACETIME.md` | MINCO quintic trajectory optimizer design, implementation, and verification |
| `PLAN_2_SFC_TRAJECTORY_GEN.md` | Safe Flight Corridor trajectory generation plan |
| `PLAN_3_CERES_SOLVER.md` | Ceres solver integration plan |
| `PARAMETER_TUNING_GUIDE.md` | Quick reference for all tunable parameters with file locations |
| `OBSTACLE_AVOIDANCE_FIX_REPORT.md` | Obstacle avoidance fix analysis (global + local) |
| `MAP_GRADIENT_FIX_REPORT.md` | Point-LIO gravity calibration fix |
| `PROBLEM_REPORT_CMD_VEL_FLUCTUATION.md` | cmd_vel fluctuation diagnosis |
| `PROBLEM_REPORT_LIDAR_TOPICS.md` | LiDAR topic configuration report |
| `PROBLEM_REPORT_LOCALIZATION_TILT.md` | Localization tilt/drift analysis |

---

## 2. What Was Removed

### 2.1 Old PCD Files (4 files, all pre-March)

| File | Reason |
|------|--------|
| `Point-LIO/PCD/Feb5_calibration.pcd` | Outdated calibration data |
| `Point-LIO/PCD/Feb5_original.pcd` | Outdated original scan |
| `hdl_graph_slam/map/Feb10.pcd` | Old map from February |
| `hdl_graph_slam/map/Feb12.pcd` | Old map from February |

### 2.2 Old Map Directories (5 directories, 22 files)

| Directory | Contents | Reason |
|-----------|----------|--------|
| `trajectory_generation/map/HIT/` | 8 PNG files (2024 maps) | HIT campus maps — not for current arena |
| `trajectory_generation/map/NewLAb/` | 3 PNG files | Old lab map |
| `trajectory_generation/map/lab/` | 3 PNG files | Old lab map |
| `trajectory_generation/map/hallway/` | 3 PNG files | Hallway test map |
| `trajectory_generation/map/old/` | 4 PNG files | Archived old maps |
| `trajectory_generation/occfinal.png` | 1 PNG file | Stale copy at wrong directory level |

### 2.3 Old Standalone Files

| File | Reason |
|------|--------|
| `HIT_code/mcu_communicator.cpp` | Replaced by `DecisionNode/src/decision_node/src/mcu_communicator.cpp` |

### 2.4 Note: D*Lite Legacy Files RETAINED

The following legacy D*Lite navigation files were **not removed** because they're part of the `bot_sim` package which is still used for other utilities (imu_filter, map_server, local_frame_bridge, map_image_publisher):

- `bot_sim/src/dstarlite.cpp`
- `bot_sim/src/dstarlite_test_while.cpp`
- `bot_sim/launch_real/dstarlite.launch`

D*Lite is **disabled by default** (`enable_dstarlite=false` in the master launch file).

---

## 3. Gitignore Updates

Added to `.gitignore`:
```
# Large binary data — keep out of version control
*.pcd
*.bag

# Catkin workspace artifacts
catkin_generated/
atomic_configure/
gtest/
test_results/
CTestConfiguration.ini
CTestCustom.cmake
```

This prevents future accidental commits of large binary files and build artifacts.

---

## 4. Repository Structure (Post-Sync)

```
RM_Sentry_2026/
├── .gitignore                (updated)
├── .gitmodules               (Sophus submodule)
├── DecisionNode/             (NEW — MCU serial bridge + decision)
│   └── src/decision_node/
├── HIT_code/
│   ├── sentry_deps/Sophus/
│   └── sentry_planning_ws/
│       ├── src/
│       │   ├── ocs2/                    (OCS2 SQP-MPC framework)
│       │   ├── ocs2_robotic_assets/
│       │   ├── sentry_msgs/
│       │   └── sentry_planning/
│       │       ├── trajectory_generation/   (A* + MINCO global planner)
│       │       ├── trajectory_tracking/     (OCS2 MPC local tracker)
│       │       └── waypoint_generator/
│       └── tools/pcd_to_maps.py
├── HIT_Integrated_test/
│   ├── HIT_intergration_test.launch    (master launch)
│   ├── HIT_pointlio_map_builder.launch
│   ├── hit_env.bash                    (env setup — relative paths)
│   ├── test.rviz
│   ├── mds/                            (NEW — 10 documentation files)
│   └── sim_nav/src/
│       ├── bot_sim/         (imu_filter, map_server, local_frame_bridge)
│       ├── FAST_LIO/        (mapping)
│       ├── hdl_localization/ (NDT localization)
│       ├── Point-LIO/       (mapping, PCD dir now empty)
│       └── ...
├── Livox-SDK2/              (LiDAR SDK)
├── ws_cloud/                (livox_cloudpoint_processor)
└── ws_livox/                (livox_ros_driver2)
```

---

## 5. Deployment Notes

### Build Order
After cloning, build the three workspaces in order:
```bash
# 1. Livox driver
cd RM_Sentry_2026/ws_livox && catkin_make

# 2. Cloud processor
cd RM_Sentry_2026/ws_cloud && catkin_make

# 3. HIT sim_nav (extends sentry_planning_ws)
cd RM_Sentry_2026/HIT_code/sentry_planning_ws && catkin_make
cd RM_Sentry_2026/HIT_Integrated_test/sim_nav && catkin_make

# 4. DecisionNode
cd RM_Sentry_2026/DecisionNode && catkin_make
```

### Path Configuration
The `hit_env.bash` now uses `$(dirname)` to resolve paths relative to the script location, making the repo self-contained. However, the master launch file (`HIT_intergration_test.launch`) contains `<env>` tags with absolute paths that may need updating for deployment on a new machine:
```xml
<env name="ROS_PACKAGE_PATH" value="/home/sentry_train_test/AstarTraining/DecisionNode/src:..." />
```

### Map Setup
1. Record a PCD map using Point-LIO: `roslaunch HIT_pointlio_map_builder.launch`
2. Convert to planning maps: `python3 tools/pcd_to_maps.py <your_map.pcd>`
3. Copy output to `trajectory_generation/map/` (bevfinal.png, occfinal.png, occtopo.png, map_meta.yaml)

### Running
```bash
source RM_Sentry_2026/HIT_Integrated_test/hit_env.bash
roslaunch HIT_intergration_test.launch
```

---

## 6. Current System Parameters (Post Fix 43)

| Parameter | Value | File |
|-----------|-------|------|
| `reference_desire_speed` | 2.0 m/s | HIT_intergration_test.launch |
| `local_v_max` | 2.0 m/s | HIT_intergration_test.launch |
| `robot_radius` | 0.35 m | HIT_intergration_test.launch |
| `Q (x,y,v,φ)` | {150, 150, 60, 80} | task.info |
| `R (accel, ω)` | {1.5, 0.15} | task.info |
| `sqpIteration` | 12 | task.info |
| `hpipmMode` | ROBUST | local_planner.cpp |
| `collision_barrier (μ, δ)` | (20.0, 1.0) | local_planner.cpp |
| `distance_threshold_static` | 0.25 | SentryRobotCollisionConstraint.h |
| `distance_threshold_dynamic` | 0.49 | SentryRobotCollisionConstraint.h |
| `MAX_LEAD_TIME` | 0.5 s | local_planner.cpp |
| `MAX_OBS_DIST` | 1.0 m | local_planner.cpp |
| `OBS_SEARCH_HALF` | 20 cells | local_planner.cpp |
| `OBS_PERSIST (tracking)` | 5 frames (0.5s) | RM_GridMap.cpp |
| `OBS_PERSIST (generation)` | 10 frames (1.0s) | RM_GridMap.cpp |

### Verified Performance (Fix 43)
| Metric | Value |
|--------|-------|
| refGap (reference-to-robot) | 0.01–0.49 m |
| maxDev (MPC deviation) | 0.27–1.17 m |
| min_obs (nearest obstacle) | 0.59–0.70 m |
| "MPC is not safe" rate | **0%** |
