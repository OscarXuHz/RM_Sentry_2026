# RM Sentry 2026 — SLAM + HIT Navigation System Architecture Report

*Comprehensive technical reference for the full autonomous navigation stack*
*Generated: 2026-03-21 | Scope: `RM_Sentry_2026/`*

---

## Table of Contents

1. [System Overview & Launch Entry Point](#1-system-overview--launch-entry-point)
2. [Hardware & Sensor Layer](#2-hardware--sensor-layer)
3. [LiDAR Driver (livox_ros_driver2)](#3-lidar-driver-livox_ros_driver2)
4. [IMU Filtering (imu_filter)](#4-imu-filtering-imu_filter)
5. [Localization (hdl_localization — UKF + NDT)](#5-localization-hdl_localization--ukf--ndt)
6. [TF Tree & Coordinate Systems](#6-tf-tree--coordinate-systems)
7. [Map Processing Pipeline (pcd_to_maps.py)](#7-map-processing-pipeline-pcd_to_mapspy)
8. [Global Path Planning (trajectory_generation)](#8-global-path-planning-trajectory_generation)
9. [Local Trajectory Tracking (tracking_node — OCS2 MPC)](#9-local-trajectory-tracking-tracking_node--ocs2-mpc)
10. [Velocity Bridge & MCU Communication](#10-velocity-bridge--mcu-communication)
11. [Decision System (DecisionNode — Behavior Tree)](#11-decision-system-decisionnode--behavior-tree)
12. [Frame Bridge & Visualization Utilities (bot_sim)](#12-frame-bridge--visualization-utilities-bot_sim)
13. [Complete ROS Topic Map](#13-complete-ros-topic-map)
14. [Complete File Inventory](#14-complete-file-inventory)
15. [Parameter Reference](#15-parameter-reference)
16. [Known Issues & Fix History Summary](#16-known-issues--fix-history-summary)

---

## 1. System Overview & Launch Entry Point

### Architecture Summary

The RM Sentry 2026 navigation stack is a ROS 1 (Noetic-compatible) autonomous navigation system for a competition sentry robot. It runs on an Intel NUC compute module and integrates:

- **Perception**: Single Livox MID-360 LiDAR (20° tilted mount) publishing PointCloud2
- **Localization**: UKF + NDT scan matching against a pre-built PCD map
- **Global Planning**: Topological PRM search + cubic spline trajectory optimization (MINCO-inspired)
- **Local Tracking**: Nonlinear MPC via OCS2 SQP solver with obstacle avoidance
- **Decision**: BehaviorTree.CPP-based strategy node integrating referee system data
- **Communication**: CRC8-validated serial protocol to STM32 MCU at 115200 baud

### Launch Entry Point

**File**: `HIT_Integrated_test/HIT_intergration_test.launch`

This master launch file brings up the entire pipeline with the following node sequence:

| Step | Component | Launch Include/Node | Default State |
|------|-----------|-------------------|---------------|
| 1 | 2D Map Server | `bot_sim/map_server.launch` | **OFF** (legacy) |
| 2 | IMU Filter | `bot_sim/imu_filter.launch` | ON |
| 3 | LiDAR Driver | `livox_ros_driver2/rviz_MID360.launch` | ON (rviz OFF) |
| 4 | Localization | `hdl_localization/hdl_localization.launch` | ON |
| 5 | TF Transform | `bot_sim/real_robot_transform.launch` | ON |
| 6 | Point Cloud Filter | `livox_cloudpoint_processor` | **OFF** (commented out) |
| 7 | D* Lite Planner | `bot_sim/dstarlite.launch` | **OFF** |
| 8 | Global Planner | `trajectory_generation/global_searcher.launch` | ON |
| 8.1 | Frame Bridge | `bot_sim/local_frame_bridge.py` | ON |
| 9 | Local Tracker | `tracking_node/trajectory_planning.launch` | ON |
| 9.5-9.7 | Map Visualization | `bot_sim/map_image_publisher.py` ×3 | ON |
| 10a | Velocity Bridge | `tracking_node/hit_bridge` | ON |
| 10b | MCU Serial | `decision_node/mcu_communicator` | ON |
| 10c | Legacy Serial | `bot_sim/ser2msg_tf_decision_givepoint.launch` | **OFF** |

### Key Launch Arguments

| Argument | Default | Purpose |
|----------|---------|---------|
| `map_meta_file` | `$(find trajectory_generation)/map/map_meta.yaml` | Central map metadata source |
| `map_resolution` | Parsed from YAML (~0.05 m) | Grid resolution |
| `robot_radius` | 0.35 m | Collision radius |
| `reference_desire_speed` | 2.0 m/s | Global planner desired speed (was 2.8) |
| `local_v_max` | 2.0 m/s | MPC velocity ceiling (was 2.8) |
| `points_topic` | `/livox/lidar` | LiDAR input topic |
| `serial_port` | `/dev/ttyACM0` | MCU serial device |
| `serial_baudrate` | 115200 | Baud rate |
| `enable_mcu_communicator` | true | MCU serial on/off |
| `enable_rviz` | false | Headless NUC — no display |
| `enable_dstarlite` | false | Legacy planner |
| `enable_map_server` | false | Legacy 2D `/map` |

### Environment Setup

```xml
<env name="ROS_PACKAGE_PATH" value="...DecisionNode/src:...sentry_planning_ws/src:$(env ROS_PACKAGE_PATH)" />
<env name="CMAKE_PREFIX_PATH" value="...DecisionNode/devel:$(env CMAKE_PREFIX_PATH)" />
```

---

## 2. Hardware & Sensor Layer

### LiDAR: Livox MID-360

| Property | Value |
|----------|-------|
| Model | Livox MID-360 |
| IP Address | 192.168.1.107 |
| Host IP | 192.168.1.5 |
| Mounting | 20° roll tilt (physical mount angle) |
| FOV | 360° × ~59° (non-repetitive scan pattern) |
| Data Format | PointCloud2 (`xfer_format=0`) |
| Publishing Rate | 10 Hz |
| Point Fields | x, y, z (float32), intensity (float32), tag (uint8), line (uint8), timestamp (float64) |
| Point Step | 26 bytes |
| IMU Rate | ~200 Hz (built into MID-360) |

### Network Configuration (from MID360_config.json)

```
LiDAR ports: cmd=56100, push=56200, point=56300, imu=56400, log=56500
Host ports:  cmd=56101, push=56201, point=56301, imu=56401, log=56501
```

### MCU: STM32 (via USB Serial)

| Property | Value |
|----------|-------|
| Port | `/dev/ttyACM0` |
| Baud Rate | 115200 |
| Protocol | Custom binary frames with CRC8 |
| Frame Types | 0x91 (MCU→NUC, 89B), 0x92 (NUC→MCU, 7B), 0x93 (NUC→MCU, 17B) |

---

## 3. LiDAR Driver (livox_ros_driver2)

### Package Location

`ws_livox/src/livox_ros_driver2/`

### Active Configuration

**Launch**: `launch_ROS1/rviz_MID360.launch` with HIT overrides:
- `rviz_enable=false` (headless NUC)
- `msg_frame_id=aft_mapped` (override from default `livox_frame`)

**Config**: `config/MID360_config.json`
```json
{
  "lidar_configs": [{
    "ip": "192.168.1.107",
    "pcl_data_type": 1,
    "pattern_mode": 0,
    "extrinsic_parameter": {
      "roll": 20.0, "pitch": 0.0, "yaw": 0.0,
      "x": 0.0, "y": 0.0, "z": 0.0
    }
  }]
}
```

The `roll: 20.0` compensates for the physical 20° tilt of the LiDAR mount. The driver applies this rotation to all point data before publishing.

### Published Topics

| Topic | Type | Frame ID | Rate | Description |
|-------|------|----------|------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `aft_mapped` | 10 Hz | Point cloud (26-byte points) |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_frame` (hardcoded) | ~200 Hz | Raw accelerometer + gyroscope |

**Important**: The IMU `frame_id` is hardcoded to `livox_frame` in `lddc.cpp:502` and is NOT overridable by the `msg_frame_id` parameter. Only the point cloud frame_id is configurable.

### Alternative Configs (Not Active)

| Config | Use Case |
|--------|----------|
| `dual_MID360_config.json` | Two MID-360s at 192.168.1.3 and 192.168.1.105 |
| `mixed_HAP_MID360_config.json` | HAP + MID-360 combo |
| `HAP_config.json` | Single HAP LiDAR |

### Custom Message Types

**CustomMsg.msg** (used when `xfer_format=1`, currently NOT active):
```
Header header
uint64 timebase        # Base timestamp (ns)
uint32 point_num
uint8 lidar_id
CustomPoint[] points   # offset_time, x, y, z, reflectivity, tag, line
```

### Point Cloud Filter (DISABLED)

**Package**: `ws_cloud/src/livox_cloudpoint_processor/`

Currently disabled in the launch file because:
- The filter (`livox_cloudpoint_processor`) expects `CustomMsg` input (`xfer_format=1`)
- The driver is configured for `PointCloud2` output (`xfer_format=0`)
- The Old_nav pipeline (working reference) does not use this filter

When active, it would provide:
- `/3Dlidar` — merged/unfiltered PointCloud2
- `/filted_topic_3d` — height-filtered PointCloud2
- `/grid` — dilated OccupancyGrid for local planning

---

## 4. IMU Filtering (imu_filter)

### Package Location

`HIT_Integrated_test/sim_nav/src/bot_sim/` (node: `imu_filter`)

### Data Flow

```
/livox/imu (raw, frame: livox_frame, ~200 Hz)
    ↓ imu_filter node
/livox/imu_filtered (transformed, ~200 Hz)
```

### Processing Steps

1. **Gravity normalization**: Multiply raw acceleration by gravity coefficient (9.81)
2. **Rotation correction**: Apply -20° rotation around X-axis to compensate for the physical LiDAR tilt
   - This matches the `roll: 20.0` in `MID360_config.json`
   - Both acceleration and angular velocity vectors are rotated
3. **Frame re-assignment**: Output retains the same frame (empty override)

### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `input_imu_topic` | `/livox/imu` | Raw input |
| `output_imu_topic` | `/livox/imu_filtered` | Filtered output |
| `gravity` | 9.81 | Gravity normalization constant |

---

## 5. Localization (hdl_localization — UKF + NDT)

### Package Location

`HIT_Integrated_test/sim_nav/hdl_localization/` (also built in `sim_nav/`)

### Algorithm: Hybrid UKF + NDT-OMP

This is a **predict-correct** estimator:

1. **Prediction (UKF)**: Uses filtered IMU (accelerometer + gyroscope) to propagate state at ~200 Hz
2. **Correction (NDT)**: Scans the incoming point cloud against a pre-built global PCD map using multi-threaded NDT (NDT-OMP), correcting the UKF state at ~5-10 Hz

### State Space (16-dimensional)

| State Variables | Description |
|----------------|-------------|
| `p = [x, y, z]` | Position in map frame |
| `v = [vx, vy, vz]` | Velocity |
| `q = [w, x, y, z]` | Orientation quaternion |
| `b_a = [bax, bay, baz]` | Accelerometer bias |
| `b_ω = [bgx, bgy, bgz]` | Gyroscope bias |

### UKF Prediction Equations

$$p_{k+1} = p_k + v_k \cdot \Delta t$$
$$v_{k+1} = v_k + (R \cdot (a_{imu} - b_a) - g) \cdot \Delta t$$
$$q_{k+1} = q_k \otimes \exp\left(\frac{1}{2}(\omega_{imu} - b_\omega) \cdot \Delta t\right)$$

### NDT-OMP Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| `reg_method` | `NDT_OMP` | Multi-threaded NDT |
| `ndt_neighbor_search_method` | `DIRECT7` | 7-cell direct neighborhood |
| `ndt_neighbor_search_radius` | 0.5 m | Search radius |
| `ndt_resolution` | 1.0 m | NDT grid cell size |
| `downsample_resolution` | 0.1 m | VoxelGrid pre-filter |

### Global Map

| Parameter | Value |
|-----------|-------|
| `globalmap_pcd` | `HIT_Integrated_test/pcd/current.pcd` |
| `downsample_resolution` | 0.1 m |
| `convert_utm_to_local` | true |

The globalmap server loads `current.pcd`, downsamples it with VoxelGrid (0.1 m leaf), and publishes to `/globalmap` (latched).

### Initial Pose

| Parameter | Value | Description |
|-----------|-------|-------------|
| `specify_init_pose` | true | Use fixed initial pose |
| `init_pos_x/y/z` | 0.0, 0.0, 0.0 | Origin |
| `init_ori_w` | 0.7017 | 90° yaw rotation quaternion |
| `init_ori_x/y` | 0.0, 0.0 | — |
| `init_ori_z` | -0.7017 | — |

### Soft Z-Constraint (Fix #33c)

```cpp
pose(2, 3) = clamp(z, init_pos_z - 0.15, init_pos_z + 0.15)
```

Prevents floor drift. The PCD map ground was normalized to z≈0, and the LiDAR is mounted ~0.3 m above ground, so z is constrained to ±0.15 m.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | PointCloud2 | Raw LiDAR scans |
| `/livox/imu_filtered` | Imu | Filtered IMU data |
| `/globalmap` | PointCloud2 | Reference PCD map |
| `/initialpose` | PoseWithCovarianceStamped | Manual pose reset (RViz) |

### Published Topics

| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/odom` | Odometry | `map` → `aft_mapped` | **Primary output**: localized pose + velocity |
| `/aligned_points` | PointCloud2 | `map` | NDT-aligned point cloud (debug) |
| `/status` | ScanMatchingStatus | — | NDT convergence/error info |
| `/globalmap` | PointCloud2 | `map` | Latched global map |

### TF Broadcast

**`map` → `aft_mapped`** at every LiDAR frame (~10 Hz)

---

## 6. TF Tree & Coordinate Systems

### Active TF Tree

```
map
 └── aft_mapped          (hdl_localization: NDT pose)
      └── gimbal_frame   (real_robot_transform: yaw-only extraction)
```

### Frame Descriptions

| Frame | Publisher | Description |
|-------|----------|-------------|
| `map` | hdl_localization | Global fixed frame, origin at PCD map origin |
| `aft_mapped` | hdl_localization | Robot body frame (6-DOF pose from NDT) |
| `gimbal_frame` | real_robot_transform | Sensor/gimbal mount frame (yaw-only extraction from aft_mapped) |
| `livox_frame` | — | IMU reference (hardcoded in driver, not in TF tree) |

### real_robot_transform Node

**Package**: `bot_sim` (C++ node)

This node:
1. Listens for `aft_mapped → map` transform
2. Extracts the **yaw-only** component (nullifies roll and pitch)
3. Applies any RPY offsets (`roll_offset_deg`, `pitch_offset_deg`, `yaw_offset_deg` — all 0.0 by default)
4. Broadcasts `aft_mapped → gimbal_frame` at 20 Hz

**Purpose**: The gimbal frame provides a level (no roll/pitch) yaw-aligned reference for the planning and tracking systems, even though the LiDAR-based localization has full 3-DOF rotation.

### Coordinate Conventions

- **X**: Forward (in map frame, depends on map orientation)
- **Y**: Left
- **Z**: Up
- All frames use right-hand coordinate system
- The IMU filter applies -20° X-rotation to align with the tilted LiDAR mount

---

## 7. Map Processing Pipeline (pcd_to_maps.py)

### Tool Location

`HIT_code/sentry_planning_ws/tools/pcd_to_maps.py`

### Input

A pre-built 3D point cloud map (`.pcd` file) from SLAM (e.g., `current.pcd`).

### Output (3 PNG maps + 1 YAML metadata)

| Output File | Type | Description |
|-------------|------|-------------|
| `occfinal.png` | Binary occupancy grid | 0=free, 255=occupied (walls/obstacles) |
| `bevfinal.png` | Height-encoded BEV map | Terrain height per cell: $z = \frac{\text{pixel} \times h_{interval}}{255} + h_{bias}$ |
| `occtopo.png` | Skeleton/topology map | Medial axis of free space (lane centerlines) |
| `map_meta.yaml` | YAML metadata | Resolution, bounds, grid dimensions |

### Processing Algorithm

```
1. Load PCD file (open3d)
2. Z-band filter: keep points in [-0.5, 0.4] m
3. Discretize to grid: world_to_grid(x, y) → (row, col)
4. Build occupancy (occfinal):
   - Per cell: compute z_max - z_min
   - wall_mask = (z_max - z_min) >= 0.5 m
   - high_mask = z_max >= (floor_height + threshold)
   - occ[wall_mask | high_mask] = 255
5. Build BEV height map (bevfinal):
   - Per cell: encode height as pixel = ((z - bias) * 255 / interval)
6. Build topology skeleton (occtopo):
   - free = (occ == 0)
   - skeleton = binary_skeletonize(free)  [scikit-image]
7. Deploy to $(find trajectory_generation)/map/
8. Write map_meta.yaml
```

### Current Map Metadata (`map_meta.yaml`)

```yaml
resolution: 0.05       # m/pixel
map_x_size: 20.0       # meters
map_y_size: 20.0       # meters
map_lower_x: -13.288   # world origin X (lower-left)
map_lower_y: -14.701   # world origin Y (lower-left)
rows: 400              # = 20.0 / 0.05
cols: 400              # = 20.0 / 0.05
```

Grid bounds in world coordinates:
- X: [-13.288, 6.712]
- Y: [-14.701, 5.299]

---

## 8. Global Path Planning (trajectory_generation)

### Package Location

`HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/`

### Architecture: Four-Stage Pipeline

```
Goal (/clicked_point)
    ↓
[Stage 1] Topological PRM → sparse waypoints (Dijkstra)
    ↓
[Stage 2] A* path refinement → collision-checked denser path
    ↓
[Stage 3] L-BFGS trajectory optimization → smooth polynomial
    ↓
[Stage 4] Velocity profiling → cubic polynomial trajectory with timing
    ↓
Output: /global_trajectory (trajectoryPoly message)
```

### Stage 1: Topological PRM Search (TopoSearch.cpp)

Inspired by **fast-planner**, this builds a sparse topological graph:

**Node Types**:
- **Guard Nodes (G)**: Exploration nodes. Two guards must be mutually invisible or far apart.
- **Connection Nodes (C)**: Link exactly two guard nodes.

**Sampling Strategy**:
- Primary samples from **skeleton centerlines** (`occtopo.png`) for efficiency
- Additional samples within 6 m radius of centerlines (LiDAR confidence range)
- Random samples in an ellipse around start→goal

**Visibility Check Algorithm**:
1. Connect candidate point `p` to guard `p2`
2. Step-sample along the segment
3. Check for obstacles AND height differences between consecutive samples
4. Height-aware passability: can descend stairs/ramps but NOT ascend

**Graph Search**: Dijkstra on the topological graph for approximate shortest path.

### Stage 2: A* Path Refinement (Astar_searcher.cpp)

Shortcuts and refines the sparse topological path:
- Collision-checks edges on the occupancy grid
- Produces a denser waypoint sequence

### Stage 3: L-BFGS Trajectory Optimization (path_smooth.cpp)

Cubic spline interpolation with MINCO-inspired control point optimization:

**Spline formulation** (per segment $i$):
$$S_i(x) = a_i + b_i(x - x_i) + c_i(x - x_i)^2 + d_i(x - x_i)^3$$

**Optimization objectives**:
- Trajectory smoothness (minimize curvature)
- Obstacle repulsion cost (L2 distance to occupied cells)
  - Search radius: 1 m around path
  - Activation radius: R = 0.3 m (only compute loss within this distance)
  - Additional 2.5 m raycast check to prevent optimizer pushing points into far obstacles

**Solver**: L-BFGS

**Note**: The current implementation only optimizes control point positions (P), NOT segment time durations (T). Full MINCO joint space-time optimization is a planned future improvement.

### Stage 4: Velocity Profiling (reference_path.cpp)

Generates time-parameterized trajectory:
- Trapezoidal velocity profile with slope awareness
- Polynomial coefficient computation for cubic segments
- All segment durations computed based on `reference_desire_speed` (2.0 m/s)

### Replanning FSM (replan_fsm.cpp)

**States**: `INIT` → `WAIT_TARGET` → `GEN_NEW_TRAJ` → `EXEC_TRAJ` / `REPLAN_TRAJ`

**Replanning triggers** (from tracking_node):
- `/replan_flag` (Bool) — collision detected or off-course
- Periodic replan every 5 seconds
- 3-second cooldown after new trajectory (suppress premature replans)

### Output Message: `trajectoryPoly`

```
float64[] coef_x      # Cubic polynomial X coefficients per segment
float64[] coef_y      # Cubic polynomial Y coefficients per segment
float64[] duration    # Time duration per segment
uint8 motion_mode     # FASTMOTION, SAFEAMFLIGHT, etc.
```

### Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/odom_local` | Odometry | Robot position (remapped from `/odometry_imu`) |
| `/clicked_point` | PointStamped | Goal from RViz or decision node |
| `/slaver/wheel_state` | slaver_speed | Wheel encoder feedback |
| `/replan_flag` | Bool | Trigger from tracking_node |
| `/slaver/robot_HP` | Custom | Sentry HP for game logic |
| `/referee/robots_hp` | RobotsHP | All robots HP |

### Published Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/global_trajectory` | trajectoryPoly | **Primary output**: polynomial trajectory |
| `/target_result` | Point | Current goal position |
| `/xtl_flag` | Bool | Spin/gyroscope mode flag |
| `/grid_map_vis` | PointCloud2 | Global map visualization |
| RViz markers | MarkerArray | Paths, nodes, connections |

### Key Global Planner Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `map_resolution` | 0.05 m | Grid cell size |
| `robot_radius` | 0.35 m | Collision buffer |
| `reference_desire_speed` | 2.0 m/s | Target velocity |
| `reference_v_max` | 2.5 m/s | Hard velocity cap |
| `reference_a_max` | 4.0 m/s² | Acceleration limit |
| `reference_w_max` | 4.0 rad/s | Angular velocity limit |
| `search_radius` | 6.0 m | LiDAR confidence radius |
| `search_height_min` | -0.05 m | Ground obstacle catch |
| `search_height_max` | 1.2 m | Ceiling filter |
| `height_threshold` | 0.08 m | Low obstacle detection |
| `height_sencond_high_threshold` | 0.2 m | Bridge/underpass detection |
| `obstacle_swell_flag` | true | Inflate obstacles by robot_radius |

### Map Files Used by Global Planner

| File | Usage |
|------|-------|
| `occfinal.png` | Static occupancy grid (walls/obstacles) |
| `bevfinal.png` | Height-encoded BEV map (terrain awareness) |
| `occtopo.png` | Skeleton centerlines (sampling efficiency) |
| `map_meta.yaml` | Grid resolution, dimensions, world origin |

### Dynamic Obstacle Handling (RM_GridMap.cpp)

The grid map maintains three layers:

| Layer | Source | Update | Purpose |
|-------|--------|--------|---------|
| `data[]` | `occfinal.png` | Static (never) | Walls and permanent obstacles |
| `l_data[]` | `/cloud_registered_world` (LiDAR) | Every frame + decay | Real-time dynamic obstacles |
| `GridNodeMap` | `bevfinal.png` | Static (never) | Per-cell height and z-occupancy |

Dynamic obstacles persist for ~5 frames then decay. Static walls in `data[]` are never overwritten.

---

## 9. Local Trajectory Tracking (tracking_node — OCS2 MPC)

### Package Location

`HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/`

### Algorithm: Nonlinear MPC with OCS2 SQP

**Robot Model**: Simplified kinematic (NOT full Ackermann):

$$\dot{x} = v \cdot \cos(\varphi)$$
$$\dot{y} = v \cdot \sin(\varphi)$$
$$\dot{v} = a$$
$$\dot{\varphi} = \omega$$

**State**: $[x, y, v, \varphi]^T$ (position, velocity, heading)
**Control**: $[a, \omega]^T$ (acceleration, angular velocity)

### MPC Formulation

**Cost function**:
$$J = \sum_{j=0}^{N} \left[ \| x_{k+j} - x_{ref,k+j} \|_Q^2 + \| u_{k+j} \|_R^2 \right] + \text{BarrierPenalty}$$

**Q matrix (state cost)**: diag([80, 80, 15, 70]) — position, position, velocity, heading
**R matrix (input cost)**: diag([1.5, 0.1]) — acceleration, angular velocity
**Terminal cost multiplier**: $\rho_N = 2.0$ (double weight on final state)

**Horizon**: 20 steps × 0.1 s = 2.0 s lookahead

### Obstacle Avoidance (Relaxed Barrier Penalty)

**Constraint formulation** (soft obstacle avoidance):
$$\| p(x_{k+j}) - o_i \|^2 > r^2$$

Implemented as relaxed log-barrier with parameters:
- $\mu = 20$ (barrier weight)
- $\delta = 1.0$ m (barrier activation distance, Fix 42b)
- Static clearance: 0.25 m² threshold (0.5 m effective)
- Dynamic clearance: 0.49 m² threshold (0.7 m effective)

**Obstacle search strategy** (Fix 42a):
- Divide 360° into 8 angular sectors
- Keep only closest obstacle per sector
- Ignore obstacles beyond 1.0 m (negligible barrier effect)
- Search around BOTH reference trajectory AND previous predicted trajectory (Fix 39a)
- Total: ~60-80 constraints per solve (down from 200+)

### Solver: OCS2 SQP with HPIPM

- Sequential Quadratic Programming
- HPIPM backend (robust mode, regularization 1e-8)
- Supports soft constraints via Lagrange multipliers

### State/Input Bounds

| Quantity | Bound | Unit |
|----------|-------|------|
| Velocity | ±2.5 | m/s |
| Acceleration | ±3.5 | m/s² |
| Angular velocity | ±6.0 | rad/s |
| Gyro-mode velocity | ±1.8 | m/s |
| Gyro-mode angular | ±4.0 | rad/s |

### Motion Modes (xtl_flag)

| Value | Mode | Trigger | Behavior |
|-------|------|---------|----------|
| 0 | Normal | Default | Forward/backward tracking |
| 1 | Reverse-ready | Short trajectory or bridge | Prepare for direction change |
| 2 | Arrived | Goal < 0.05 m | All speeds zero |
| 3 | Gyroscope | Outpost HP < 1 | Spinning chassis while moving |

### Anti-Stuck Detection (checkMotionNormal)

**Trigger conditions** (Fix 35d):
- Stuck duration > 2.0 s
- `max_speed < 0.15 m/s`
- `average_speed < 0.05 m/s`
- Not already at goal

**Action**: Reverse at half speed with hard turn for 0.5-0.6 s, then trigger global replan.

### Three-Stage Replanning Architecture

```
Stage 1: NMPC Dynamic Avoidance (soft barrier)
    ↓ (collision still predicted?)
Stage 2: Local Trajectory Safety Check
    → checkfeasible(): Verify steps 1-5 don't hit dynamic obstacles
    → If collision: set /replan_flag
    ↓
Stage 3: Global Replan
    → trajectory_generation receives /replan_flag
    → Full topological re-search
```

**Additional replan triggers**:
- Off-course > 0.8 m for > 4 frames (Fix 38a)
- Periodic every 5 seconds (Fix 38b)
- 3-second cooldown after new trajectory (Fix 42c)

### Output: `/sentry_des_speed` (slaver_speed)

The MPC output is decomposed into body-frame velocities:

```cpp
Δφ = φ_target - φ_lidar  // Heading difference
vx = v · cos(Δφ)         // Body-frame forward
vy = v · sin(Δφ)         // Body-frame lateral
```

Message fields:
```
angle_target  = vx (body-frame forward velocity)
angle_current = vy (body-frame lateral velocity)
line_speed    = acceleration or ω
xtl_flag      = motion mode (0/1/2/3)
in_bridge     = bridge crossing flag
```

### Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/odom_local` (remapped from `/odometry_imu`) | Odometry | Robot state — **triggers MPC solve** |
| `/global_trajectory` | trajectoryPoly | Reference from global planner |
| `/slaver/wheel_state` | slaver_speed | Wheel encoder (overrides LiDAR velo when available) |

### Published Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/sentry_des_speed` | slaver_speed | **Primary output**: body-frame vx, vy, ω |
| `/replan_flag` | Bool | Trigger global replanning |
| RViz markers | MarkerArray | Predicted trajectory, obstacles |

### Key Tracking Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `planning_horizon` | 20 steps | MPC prediction steps |
| `dt` | 0.1 s | Time step (2.0 s total) |
| `rho_` | 1.0 | Running cost multiplier |
| `rhoN_` | 2.0 | Terminal cost multiplier |
| `local_v_max` | 2.0 m/s | Velocity cap |
| `local_a_max` | 3.5 m/s² | Acceleration cap |
| `local_w_max` | 8.0 rad/s | Angular velocity cap |
| `search_radius` | 6.0 m | Obstacle search radius |
| `robot_radius` | 0.35 m | Collision buffer |

---

## 10. Velocity Bridge & MCU Communication

### hit_bridge Node

**Package**: `tracking_node`
**Source**: `src/hit_bridge.cpp`

Direct passthrough from HIT planning output to standard ROS Twist:

```cpp
// /sentry_des_speed → /cmd_vel
twist.linear.x  = slaver_speed.angle_target   // vx
twist.linear.y  = slaver_speed.angle_current   // vy
twist.angular.z = slaver_speed.line_speed       // ω
```

**Arrival detection**: When $|vx| + |vy| + |\omega| < 0.01$, publishes `true` on `/dstar_status`.

### mcu_communicator Node

**Package**: `decision_node`
**Source**: `DecisionNode/src/decision_node/src/mcu_communicator.cpp`

This node bridges ROS and the STM32 MCU over serial.

### Serial Protocol Specification

#### Frame 1: MCUDataFrame (0x91) — MCU → NUC, 89 bytes

```
Byte  | Field              | Type    | Description
------+--------------------+---------+---------------------------
0     | sof                | uint8   | 0x91 start-of-frame
1-4   | yaw_angle          | float   | Gimbal yaw (radians)
5-8   | chassis_imu        | float   | Chassis IMU (radians)
9     | motion_mode        | uint8   | 0=attack, 1=defend, 2=move, 3=brake
10-13 | operator_x         | float   | Operator position X
14-17 | operator_y         | float   | Operator position Y
18    | robot_id           | uint8   | Robot ID
19    | robot_color        | uint8   | 0=red, 1=blue
20    | game_progress      | uint8   | Game stage
21-22 | red_1_hp           | uint16  | Red hero HP
23-24 | red_3_hp           | uint16  | Red infantry-3 HP
25-26 | red_7_hp           | uint16  | Red sentry HP
27-28 | blue_1_hp          | uint16  | Blue hero HP
29-30 | blue_3_hp          | uint16  | Blue infantry-3 HP
31-32 | blue_7_hp          | uint16  | Blue sentry HP
33-34 | red_dead           | uint16  | Red death bitmap
35-36 | blue_dead          | uint16  | Blue death bitmap
37-38 | self_hp            | uint16  | Self health
39-40 | self_max_hp        | uint16  | Max health
41-42 | bullet_remain      | uint16  | Remaining ammo
43    | occupy_status      | uint8   | 0=none,1=friendly,2=enemy,3=both
44-47 | enemy_hero_x       | float   | Enemy hero X (-8888=invalid)
48-51 | enemy_hero_y       | float   | Enemy hero Y
52-55 | enemy_engineer_x   | float   | Enemy engineer X
56-59 | enemy_engineer_y   | float   | Enemy engineer Y
60-63 | enemy_standard_3_x | float   | Enemy infantry-3 X
64-67 | enemy_standard_3_y | float   | Enemy infantry-3 Y
68-71 | enemy_standard_4_x | float   | Enemy infantry-4 X
72-75 | enemy_standard_4_y | float   | Enemy infantry-4 Y
76-79 | enemy_sentry_x     | float   | Enemy sentry X
80-83 | enemy_sentry_y     | float   | Enemy sentry Y
84    | suggested_target   | uint8   | Radar-suggested target
85-86 | radar_flags        | uint16  | Radar detection flags
87    | crc8               | uint8   | CRC8 checksum
88    | eof                | uint8   | 0xFE end-of-frame
```

**Invalid coordinate handling**: When enemy position is -8888, the last valid position is cached and re-used.

#### Frame 2: MotionCommandFrame (0x92) — NUC → MCU, 7 bytes

```
Byte | Field          | Type   | Description
-----+----------------+--------+------------------
0    | sof            | uint8  | 0x92
1    | motion_mode_up | uint8  | 0=attack,1=defend,2=move,3=brake
2    | hp_up          | uint8  | 0=no heal, 1=heal
3    | bullet_up      | uint8  | 0=no ammo, 1=buy ammo
4    | bullet_num     | uint8  | Ammo quantity
5    | crc8           | uint8  | CRC8
6    | eof            | uint8  | 0xFE
```

#### Frame 3: NavigationFrame (0x93) — NUC → MCU, 17 bytes

```
Byte  | Field    | Type   | Description
------+----------+--------+------------------
0     | sof      | uint8  | 0x93
1-4   | vx       | float  | Velocity X
5-8   | vy       | float  | Velocity Y
9-12  | z_angle  | float  | Angular velocity
13    | received | uint8  | Nav received flag
14    | arrived  | uint8  | Nav arrival flag
15    | crc8     | uint8  | CRC8
16    | eof      | uint8  | 0xFE
```

**Navigation sending rate**: 50 Hz (fixed timer)

### CRC8 Implementation

Uses a 256-byte lookup table with initial value 0xFF.

### mcu_communicator Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/referee/game_progress` | UInt8 | Game stage |
| `/referee/remain_hp` | UInt16 | Self HP |
| `/referee/bullet_remain` | UInt16 | Ammo count |
| `/referee/occupy_status` | UInt8 | Zone occupation |
| `/referee/red_*_hp` | UInt16 | Red team HP (hero, inf3, sentry) |
| `/referee/blue_*_hp` | UInt16 | Blue team HP |
| `/referee/red_dead`, `/referee/blue_dead` | UInt16 | Death bitmaps |
| `/referee/friendly_score`, `/referee/enemy_score` | Int32 | Score tracking |
| `/robot/robot_id`, `/robot/robot_color` | UInt8 | Robot identity |
| `/robot/self_hp`, `/robot/self_max_hp` | UInt16 | Health |
| `/mcu/yaw_angle`, `/mcu/chassis_imu` | Float32 | Gimbal/chassis angles |
| `/enemy/hero_position` | Point | Enemy hero XY |
| `/enemy/engineer_position` | Point | Enemy engineer XY |
| `/enemy/standard_3_position` | Point | Enemy infantry-3 XY |
| `/enemy/standard_4_position` | Point | Enemy infantry-4 XY |
| `/enemy/sentry_position` | Point | Enemy sentry XY |
| `/radar/suggested_target` | UInt8 | Radar suggestion |
| `/radar/radar_flags` | UInt16 | Radar flags |

### mcu_communicator Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands → NavigationFrame (0x93) |
| `/dstar_status` | Bool | Arrival flag |
| `/nav_received` | UInt8 | Navigation received flag |
| `/motion` | UInt8 | Motion mode → MotionCommandFrame (0x92) |
| `/recover` | UInt8 | Heal request |
| `/bullet_up` | UInt8 | Ammo purchase |
| `/bullet_num` | UInt8 | Ammo quantity |

---

## 11. Decision System (DecisionNode — Behavior Tree)

### Package Location

`DecisionNode/src/decision_node/`

### Architecture

Uses **BehaviorTree.CPP** framework with an XML-defined strategy tree.

### Strategy Tree (`config/strategy_tree.xml`)

The tree contains ~20 action/condition states:

| Node | Type | Purpose |
|------|------|---------|
| `INIT` | Action | System initialization |
| `PUSH` | Action | Aggressive push toward enemy |
| `OCCUPY` | Action | Capture/hold strategic positions |
| `SUPPLY` | Action | Navigate to supply station |
| `RESPAWN` | Action | Post-respawn behavior |
| `RADICAL` | Action | High-risk aggressive play |
| `WAITFOROP` | Action | Wait for operator command |
| `CentralOccupiable` | Condition | Center position occupancy check |
| `MotionChange` | Action | Change motion mode (attack/defend/move) |
| `RecoverChange` | Action | Trigger heal/ammo purchase |
| `Chase` | Action | Pursue enemy target |

### Source Files

| File | Purpose |
|------|---------|
| `strategy_node.cpp` | BT orchestration, referee/nav state, core conditions |
| `motion_change.cpp` | Motion state machine nodes |
| `central_occupiable.cpp` | Central position occupation logic |
| `recover_change.cpp` | Healing/ammo resupply nodes |
| `chase.cpp` | Enemy targeting/pursuit logic |

### Integration with Navigation

The decision system:
1. Receives referee data from `/referee/*` topics (via mcu_communicator)
2. Receives enemy positions from `/enemy/*` topics
3. Publishes goal points to `/clicked_point` (consumed by global planner)
4. Sends motion mode commands to `/motion` (forwarded to MCU via 0x92 frame)
5. Manages heal/ammo requests via `/recover`, `/bullet_up`

---

## 12. Frame Bridge & Visualization Utilities (bot_sim)

### local_frame_bridge.py

**Purpose**: Coordinate normalization between map-frame localization and local-frame planners.

**Current config** (passthrough mode):
```yaml
center_on_first_odom: false
passthrough: true
```

When `passthrough=true`, messages are forwarded without offset — effectively a topic renaming bridge.

**Topic remapping**:
| Input | Output |
|-------|--------|
| `/odom` | `/odom_local` |
| `/move_base_simple/goal` | `/clicked_point` |
| `/aligned_points` | `/aligned_points_local` |
| `/filted_topic_3d` | `/filted_topic_3d_local` |

### map_image_publisher.py (×3 instances)

Publishes PNG map images as `nav_msgs/OccupancyGrid` for RViz visualization:

| Instance | Image | Topic | Invert |
|----------|-------|-------|--------|
| `bev_image_publisher` | `bevfinal.png` | `~grid` | true (bright=occupied) |
| `occ_image_publisher` | `occfinal.png` | `~grid` | false (dark=occupied) |
| `topo_image_publisher` | `occtopo.png` | `~grid` | false |

All publish at 1 Hz in `map` frame with resolution and origin from launch args.

### Legacy Components (Default OFF)

| Component | Launch | Purpose |
|-----------|--------|---------|
| D* Lite | `dstarlite.launch` | Incremental path planner (replaced by HIT) |
| DWA | `dwa_optimizer.launch` | Dynamic Window Approach local planner |
| TEB | `teb_local_planner.launch` | Timed Elastic Band planner |
| map_server | `map_server.launch` | 2D OccupancyGrid from YAML+PGM |
| ser2msg | `ser2msg_tf_decision_givepoint.launch` | Old serial bridge with gimbal TF |

---

## 13. Complete ROS Topic Map

### Data Flow Diagram

```
                            ┌──────────────────┐
                            │  Livox MID-360    │
                            │  (192.168.1.107)  │
                            └─────┬──────┬──────┘
                                  │      │
                     /livox/lidar │      │ /livox/imu
                     (PointCloud2)│      │ (Imu, 200 Hz)
                          10 Hz  │      │
                                 │      ▼
                                 │  ┌──────────┐
                                 │  │imu_filter│ (-20° rotation)
                                 │  └────┬─────┘
                                 │       │ /livox/imu_filtered
                                 │       │
                                 ▼       ▼
                         ┌─────────────────────┐
                         │  hdl_localization    │
                         │  (UKF + NDT-OMP)     │
                         │  PCD: current.pcd    │
                         └───┬──────┬───────────┘
                             │      │
               /odom         │      │ TF: map→aft_mapped
               (Odometry)    │      │
                             │      ▼
                             │  ┌────────────────────┐
                             │  │real_robot_transform │
                             │  │ (yaw extraction)    │
                             │  └────┬───────────────┘
                             │       │ TF: aft_mapped→gimbal_frame
                             │       ▼
                             ▼
                      ┌──────────────────┐
                      │local_frame_bridge│ (passthrough mode)
                      └──┬──────┬───────┘
                         │      │
          /odom_local    │      │ /clicked_point
          (Odometry)     │      │ (PointStamped)
                         │      │
           ┌─────────────┘      └──────────────┐
           ▼                                    │
   ┌───────────────────┐              ┌────────────────────┐
   │ trajectory_tracking│◄────────────│ trajectory_generation│
   │ (OCS2 SQP-MPC)    │             │ (Topo PRM + L-BFGS) │
   │                    │  /global_   │                      │
   │                    │  trajectory │                      │
   └────────┬───────────┘  (Poly)    └──────────────────────┘
            │                           ▲         ▲
            │ /sentry_des_speed         │         │
            │ (slaver_speed)            │ /replan │ /clicked_point
            ▼                           │ _flag   │
    ┌───────────┐                       │         │
    │ hit_bridge│───────────────────────┘         │
    └─────┬─────┘                                 │
          │ /cmd_vel (Twist)                      │
          ▼                                       │
   ┌──────────────────┐     /referee/*      ┌────┴──────────┐
   │ mcu_communicator  │────────────────────►│ strategy_node │
   │ (Serial 0x93)     │     /enemy/*       │ (BehaviorTree)│
   │                   │◄───────────────────│               │
   └─────┬─────────────┘                    └───────────────┘
         │ Serial (115200 baud)                   │ /motion
         ▼                                        │ /recover
   ┌───────────┐                                  ▼
   │ STM32 MCU │◄─────────────────────  mcu_communicator
   │           │     (0x92 MotionCmd)
   └───────────┘
```

### All Active Topics

| Topic | Type | Publisher | Subscriber(s) | Rate |
|-------|------|-----------|----------------|------|
| `/livox/lidar` | PointCloud2 | livox_ros_driver2 | hdl_localization | 10 Hz |
| `/livox/imu` | Imu | livox_ros_driver2 | imu_filter | ~200 Hz |
| `/livox/imu_filtered` | Imu | imu_filter | hdl_localization | ~200 Hz |
| `/globalmap` | PointCloud2 | hdl_localization (globalmap_server) | hdl_localization | Latched |
| `/odom` | Odometry | hdl_localization | local_frame_bridge | ~10 Hz |
| `/aligned_points` | PointCloud2 | hdl_localization | local_frame_bridge | ~10 Hz |
| `/odom_local` | Odometry | local_frame_bridge | trajectory_generation, tracking_node | ~10 Hz |
| `/clicked_point` | PointStamped | local_frame_bridge / strategy_node | trajectory_generation | Event |
| `/global_trajectory` | trajectoryPoly | trajectory_generation | tracking_node | Event |
| `/sentry_des_speed` | slaver_speed | tracking_node | hit_bridge | ~50 Hz |
| `/cmd_vel` | Twist | hit_bridge | mcu_communicator | ~50 Hz |
| `/dstar_status` | Bool | hit_bridge | mcu_communicator | Event |
| `/replan_flag` | Bool | tracking_node | trajectory_generation | Event |
| `/xtl_flag` | Bool | trajectory_generation | tracking_node | Event |
| `/referee/*` (15+ topics) | Various | mcu_communicator | strategy_node | ~100 Hz |
| `/enemy/*_position` (×5) | Point | mcu_communicator | strategy_node | ~100 Hz |
| `/motion` | UInt8 | strategy_node | mcu_communicator | Event |
| `/recover`, `/bullet_up` | UInt8 | strategy_node | mcu_communicator | Event |
| `~grid` (×3 viz) | OccupancyGrid | map_image_publisher | RViz | 1 Hz |

### TF Frames

| Parent | Child | Publisher | Rate |
|--------|-------|-----------|------|
| `map` | `aft_mapped` | hdl_localization | ~10 Hz |
| `aft_mapped` | `gimbal_frame` | real_robot_transform | 20 Hz |

---

## 14. Complete File Inventory

### Core Workspaces

| Workspace | Path | Contents |
|-----------|------|----------|
| ws_livox | `RM_Sentry_2026/ws_livox/` | livox_ros_driver2 package |
| ws_cloud | `RM_Sentry_2026/ws_cloud/` | livox_cloudpoint_processor (disabled) |
| sim_nav | `RM_Sentry_2026/HIT_Integrated_test/sim_nav/` | bot_sim, hdl_localization, hdl_graph_slam, ndt_omp, etc. |
| sentry_planning_ws | `RM_Sentry_2026/HIT_code/sentry_planning_ws/` | trajectory_generation, trajectory_tracking, sentry_msgs |
| DecisionNode | `RM_Sentry_2026/DecisionNode/` | decision_node (strategy + mcu_communicator) |

### Package File Lists

#### livox_ros_driver2
```
ws_livox/src/livox_ros_driver2/
├── config/
│   ├── MID360_config.json          ← Active single-LiDAR config
│   ├── dual_MID360_config.json     ← Dual LiDAR (not used)
│   ├── mixed_HAP_MID360_config.json
│   └── HAP_config.json
├── launch_ROS1/
│   ├── rviz_MID360.launch          ← Active launch file
│   ├── msg_MID360.launch           ← CustomMsg variant
│   └── ...
├── msg/
│   ├── CustomMsg.msg
│   └── CustomPoint.msg
└── src/
    └── lddc.cpp                    ← Publisher implementation
```

#### bot_sim
```
HIT_Integrated_test/sim_nav/src/bot_sim/
├── launch_real/
│   ├── imu_filter.launch
│   ├── real_robot_transform.launch
│   ├── map_server.launch
│   ├── dstarlite.launch
│   └── ser2msg_tf_decision_givepoint.launch
├── scripts/
│   ├── local_frame_bridge.py
│   ├── map_image_publisher.py
│   └── patrol.py
└── src/
    ├── imu_filter.cpp
    ├── real_robot_transform.cpp
    ├── dstarlite.cpp
    └── ser2msg_decision_givepoint.cpp
```

#### hdl_localization
```
HIT_Integrated_test/sim_nav/hdl_localization/
├── launch/
│   └── hdl_localization.launch
├── apps/
│   ├── hdl_localization_nodelet.cpp
│   └── globalmap_server_nodelet.cpp
├── src/hdl_localization/
│   └── pose_estimator.cpp
└── include/
    ├── hdl_localization/
    │   ├── pose_estimator.hpp
    │   ├── pose_system.hpp
    │   └── odom_system.hpp
    └── kkl/alg/
        └── unscented_kalman_filter.hpp
```

#### trajectory_generation (Global Planner)
```
HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation/
├── launch/
│   ├── global_searcher.launch      ← Active launch
│   ├── global_searcher_sim.launch
│   └── global_searcher_debug.launch
├── map/
│   ├── map_meta.yaml
│   ├── occfinal.png
│   ├── bevfinal.png
│   └── occtopo.png
├── src/
│   ├── trajectory_generator_node.cpp
│   ├── replan_fsm.cpp              ← FSM state machine
│   ├── plan_manager.cpp            ← Orchestrator
│   ├── TopoSearch.cpp              ← Topological PRM
│   ├── Astar_searcher.cpp          ← A* refinement
│   ├── path_smooth.cpp             ← L-BFGS optimization
│   ├── reference_path.cpp          ← Velocity profiling
│   ├── RM_GridMap.cpp              ← Map handling + dynamic obstacles
│   └── visualization_utils.cpp
└── include/
    ├── replan_fsm.h
    ├── plan_manager.h
    ├── TopoSearch.h
    ├── Astar_searcher.h
    ├── path_smooth.h
    ├── reference_path.h
    ├── RM_GridMap.h
    └── node.h
```

#### trajectory_tracking (Local Tracker)
```
HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/
├── launch/
│   └── trajectory_planning.launch
├── src/
│   ├── tracking_node.cpp           ← Entry point
│   ├── hit_bridge.cpp              ← slaver_speed → Twist bridge
│   └── ocs2_sentry/
│       ├── SentryRobotInterface.cpp
│       ├── dynamics/
│       │   └── SentryRobotDynamtics.cpp
│       └── constraint/
│           ├── SentryRobotStateInputConstraint.cpp
│           └── SentryRobotCollisionConstraint.cpp
└── include/
    ├── tracking_manager.h          ← Main control loop
    ├── local_planner.h             ← OCS2 wrapper + obstacle search
    └── ocs2_sentry/
        ├── SentryRobotInterface.h
        ├── dynamics/SentryRobotDynamtics.h
        └── constraint/
            ├── SentryRobotStateInputConstraint.h
            └── SentryRobotCollisionConstraint.h
```

#### decision_node
```
DecisionNode/src/decision_node/
├── config/
│   ├── strategy_tree.xml           ← BT definition
│   └── backup_tree.xml
├── launch/
│   ├── mcu_communicator.launch
│   └── mcu_communicator_with_decision.launch
├── src/
│   ├── strategy_node.cpp
│   ├── mcu_communicator.cpp
│   ├── motion_change.cpp
│   ├── central_occupiable.cpp
│   ├── recover_change.cpp
│   └── chase.cpp
└── include/decision_node/
    ├── mcu_comm.hpp
    ├── motion_change.hpp
    ├── central_occupiable.hpp
    ├── recover_change.hpp
    └── chase.hpp
```

#### sentry_msgs
```
HIT_code/sentry_planning_ws/src/sentry_msgs/
└── msg/
    └── referee_system/
        ├── slaver_speed.msg
        ├── RobotsHP.msg
        └── RobotStatus.msg
```

#### Map & PCD Files
```
HIT_Integrated_test/pcd/
├── current.pcd             ← Active globalmap for localization
└── current1.pcd            ← Backup
```

#### Tools
```
HIT_code/sentry_planning_ws/tools/
└── pcd_to_maps.py          ← PCD→PNG map generator
```

---

## 15. Parameter Reference

### Critical Tuning Parameters

| Parameter | Location | Value | Impact |
|-----------|----------|-------|--------|
| `ndt_resolution` | hdl_localization.launch | 1.0 m | NDT grid cell size (smaller=more precise, slower) |
| `downsample_resolution` | hdl_localization.launch | 0.1 m | VoxelGrid filter (smaller=denser, slower) |
| `reference_desire_speed` | HIT_intergration_test.launch | 2.0 m/s | Max global trajectory speed |
| `local_v_max` | HIT_intergration_test.launch | 2.0 m/s | MPC velocity ceiling |
| `robot_radius` | HIT_intergration_test.launch | 0.35 m | Collision buffer for all planners |
| `map_resolution` | map_meta.yaml | 0.05 m | Occupancy grid cell size |
| `planning_horizon` | trajectory_planning.launch | 20 steps | MPC lookahead (×0.1s = 2.0s) |
| Q matrix | SentryRobotInterface | [80,80,15,70] | State cost: [x,y,v,φ] |
| R matrix | SentryRobotInterface | [1.5,0.1] | Input cost: [a,ω] |
| Barrier μ | SentryRobotCollisionConstraint | 20 | Obstacle penalty weight |
| Barrier δ | SentryRobotCollisionConstraint | 1.0 m | Barrier activation distance |
| Static clearance | SentryRobotCollisionConstraint | 0.25 m² | Static obstacle threshold |
| Dynamic clearance | SentryRobotCollisionConstraint | 0.49 m² | Dynamic obstacle threshold |
| `height_threshold` | global_searcher.launch | 0.08 m | Low obstacle detection (BEV) |
| `search_radius` | global_searcher.launch | 6.0 m | LiDAR confidence / sampling radius |
| `nav_frequency` | mcu_communicator | 50 Hz | Serial navigation send rate |
| Z-clamp range | hdl_localization_nodelet.cpp | ±0.15 m | Localization Z-drift prevention |
| Anti-stuck timeout | tracking_manager | 2.0 s | Before triggering reverse escape |
| Off-course threshold | local_planner | 0.8 m | Before triggering replan |
| Replan cooldown | local_planner | 3.0 s | After new trajectory |
| Periodic replan | local_planner | 5.0 s | Regular replan interval |

### slaver_speed Message Definition

```
float32 line_speed       # Forward/linear speed or acceleration
float32 angle_target     # Body-frame vx velocity
float32 angle_current    # Body-frame vy velocity
uint8 xtl_flag           # Motion mode: 0=normal, 1=reverse-ready, 2=arrived, 3=gyro
uint8 in_bridge          # Bridge crossing flag
```

---

## 16. Known Issues & Fix History Summary

### Active Design Decisions

1. **Point cloud filter disabled**: `livox_cloudpoint_processor` expects CustomMsg but driver is in PointCloud2 mode. No runtime point cloud filtering is applied — NDT and planners receive raw point clouds.

2. **Velocity reduced from 2.8 → 2.0 m/s**: Both `reference_desire_speed` and `local_v_max` were reduced to prevent wall crashes.

3. **D* Lite disabled**: Replaced by HIT topological PRM + MPC pipeline.

4. **Legacy ser2msg disabled**: Replaced by hit_bridge + mcu_communicator.

### Critical Fix History (from PLANNING_FIX_HISTORY.md)

| Fix | Description | Component |
|-----|-------------|-----------|
| #33c | Soft Z-constraint (±0.15 m clamp) in localization | hdl_localization |
| #35d | Anti-stuck detection (tight thresholds: 2.0s, 0.15/0.05 m/s) | tracking_node |
| #38a | Off-course replan trigger (>0.8 m for >4 frames) | tracking_node |
| #38b | Periodic 5-second replan for cleared obstacles | tracking_node |
| #39a | Dual trajectory obstacle search (reference + predicted) | tracking_node |
| #42a | Sector-based obstacle selection (8 sectors, closest per sector) | tracking_node |
| #42b | Barrier parameters (μ=20, δ=1.0 m) | tracking_node |
| #42c | 3-second replan cooldown after new trajectory | tracking_node |

### Documented Reports

See [README.md](README.md) for the full index of fix reports, problem reports, and system reference documentation in this directory.

---

## End-to-End Data Flow Summary

```
[Physical World]
       ↓
  Livox MID-360 (192.168.1.107, 20° tilt)
       ↓
  livox_ros_driver2 → /livox/lidar (PointCloud2, 10Hz) + /livox/imu (200Hz)
       ↓                                    ↓
       ↓                            imu_filter (-20° rotation)
       ↓                                    ↓
       ↓                          /livox/imu_filtered
       ↓                                    ↓
  hdl_localization (UKF predict @ 200Hz + NDT correct @ 10Hz)
       ↓                ↓
  /odom (Odometry)   TF: map→aft_mapped
       ↓                ↓
       ↓          real_robot_transform → TF: aft_mapped→gimbal_frame
       ↓
  local_frame_bridge (passthrough)
       ↓              ↓
  /odom_local    /clicked_point (goals)
       ↓              ↓
  trajectory_generation (Topo PRM → A* → L-BFGS → velocity profile)
       ↓
  /global_trajectory (cubic polynomial segments)
       ↓
  trajectory_tracking (OCS2 SQP-MPC, 20-step horizon)
       ↓
  /sentry_des_speed (body-frame vx, vy, ω)
       ↓
  hit_bridge → /cmd_vel (Twist)
       ↓
  mcu_communicator → Serial 0x93 NavigationFrame @ 50 Hz
       ↓
  STM32 MCU → Motor controllers → Robot motion
```
