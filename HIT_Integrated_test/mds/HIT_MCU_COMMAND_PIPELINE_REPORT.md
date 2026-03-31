# HIT Sentry — MCU Command Output Pipeline Report

*Updated: 2026-03-21 (world-frame vx/vy, frameless USB CDC, deg→rad conversion)*

---

## 1. Pipeline Overview

The navigation command pipeline sends **world-frame (map-frame) vx/vy** velocities
from the NMPC tracker to the MCU (C板) over serial UART. No angular velocity and
no gimbal-yaw decomposition. The MCU handles chassis rotation autonomously.

The MCU sends back **only the gimbal yaw angle** (world frame).

```
┌──────────────────────────────────────────────────────────────┐
│                 NMPC Solver (OCS2/SQP)                       │
│   Output: predict_input = (v, φ, a, ω)                      │
│     v = scalar speed [m/s]                                   │
│     φ = desired world heading [rad]  (wrapped coordinates)   │
│     a = acceleration, ω = angular velocity (unused below)    │
└──────────────────┬───────────────────────────────────────────┘
                   │
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  tracking_manager::rcvLidarIMUPosCallback()                  │
│  FILE: trajectory_tracking/src/tracking_manager.cpp :414     │
│                                                              │
│   world_heading = phi_ctrl + robot_add_yaw                   │
│   vx = v · cos(world_heading)   ← world-frame X vel         │
│   vy = v · sin(world_heading)   ← world-frame Y vel         │
│                                                              │
│   MPC_Control = (vx, vy, 0.0, mode)                         │
│                      │                                       │
│   publishSentryOptimalSpeed(MPC_Control)                     │
└──────────────────┬───────────────────────────────────────────┘
                   │  Topic: /sentry_des_speed
                   │  Type:  sentry_msgs/slaver_speed
                   │  Fields:
                   │    angle_target  = world vx  [m/s]
                   │    angle_current = world vy  [m/s]
                   │    line_speed    = 0.0 (unused)
                   │    xtl_flag      = mode (0–3, voted)
                   │    in_bridge     = bool
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  hit_bridge  (node)                                          │
│  FILE: trajectory_tracking/src/hit_bridge.cpp                │
│                                                              │
│   speedCallback():                                           │
│     twist.linear.x  = angle_target   → world vx             │
│     twist.linear.y  = angle_current  → world vy             │
│     twist.angular.z = 0.0            (not used)              │
│                                                              │
│   Also publishes arrival flag (vx & vy both < 0.01).        │
└──────────────────┬───────────────────────────────────────────┘
                   │  Topic: /cmd_vel          (geometry_msgs/Twist)
                   │  Topic: /dstar_status     (std_msgs/Bool)
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  mcu_communicator  (node)                                    │
│  FILE: DecisionNode/src/decision_node/src/mcu_communicator   │
│        .cpp                                                  │
│                                                              │
│   cmdVelCallback():                                          │
│     current_nav_vx_ = twist.linear.x  [m/s]                 │
│     current_nav_vy_ = twist.linear.y  [m/s]                 │
│                                                              │
│   navigationTimerCallback() @ 50 Hz:                         │
│     sendNavigationCommand(vx, vy)                            │
│       → raw 8 bytes {float vx, float vy} on serial           │
└──────────────────┬───────────────────────────────────────────┘
                   │  USB CDC: /dev/ttyUSB0 @ 115200 baud
                   │  Protocol: raw NavigationFrame (8 bytes, no framing)
                   ▼
┌──────────────────────────────────────────────────────────────┐
│  MCU (C板 STM32)  — USB CDC_Receive_FS()                     │
│  memcpy(&vision_rx, Buf, sizeof(vision_rx_t));               │
│  vision_rx_t { float v; float w; } — 8 bytes packed         │
│  v = world vx,  w = world vy                                 │
│                                                              │
│  Sends back: raw float gimbal yaw (4 bytes, DEGREES)         │
│  mcu_communicator converts deg→rad before publishing          │
│  Chassis rotation managed by MCU independently               │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. Node & File Reference

| Node | Package | Source File | Role |
|------|---------|-------------|------|
| `tracking_node` | `tracking_node` | `trajectory_tracking/src/tracking_node.cpp` | Entry point, spins `tracking_manager` |
| — | — | `trajectory_tracking/src/tracking_manager.cpp` | NMPC loop, world-frame vx/vy decomposition |
| — | — | `trajectory_tracking/include/tracking_manager.h` | Publisher/subscriber declarations |
| `hit_bridge` | `tracking_node` | `trajectory_tracking/src/hit_bridge.cpp` | `slaver_speed` → `Twist` passthrough |
| `mcu_communicator` | `decision_node` | `DecisionNode/src/decision_node/src/mcu_communicator.cpp` | USB CDC bridge: raw vx/vy TX, raw yaw RX |
| — | — | `DecisionNode/src/decision_node/include/decision_node/mcu_comm.hpp` | Frameless structs (NavigationFrame 8B, MCUDataFrame 4B) |
| — | `sentry_msgs` | `sentry_msgs/msg/referee_system/slaver_speed.msg` | Intermediate message definition |

---

## 3. Variable Glossary — Detailed Explanation

All these variables appear in `tracking_manager::rcvLidarIMUPosCallback()`:

### `robot_lidar_yaw`
- **What**: The raw yaw angle extracted from the LiDAR/IMU odometry quaternion.
- **Source**: `atan2f(2(wz+xy), 1−2(y²+z²))` from `/odom_local` (= `/odom` remapped).
- **Range**: `[−π, +π]` — this is the standard atan2 output.
- **Frame**: World (map) frame. This is the orientation of the LiDAR sensor
  (= gimbal frame, since the LiDAR is mounted on the gimbal).

### `robot_cur_yaw`  (the wrapped yaw)
- **What**: The yaw used as the MPC state variable, wrapped into `[−π, +π]`.
- **Source**: When `has_wheel_state_` is true, uses wheel encoder yaw; otherwise
  falls back to `robot_lidar_yaw`. Then wrapping is applied:
  ```
  k = floor(raw_yaw / 2π)        → strips full rotations
  robot_yaw_temp = raw_yaw − k·2π
  j = floor(robot_yaw_temp / π)  → wraps into [−π, π]
  robot_cur_yaw = robot_yaw_temp − j·2π
  ```
- **Purpose**: The NMPC solver operates in `[−π, π]` to avoid discontinuities at ±π
  causing numerical issues. The "true" world yaw = `robot_cur_yaw + robot_add_yaw`.

### `robot_add_yaw`
- **What**: The 2kπ offset removed during yaw wrapping.
- **Value**: `(k + j) · 2π` where k and j are the integer divisors from wrapping.
- **Purpose**: Bookkeeping to recover the original unwrapped world yaw from the
  wrapped MPC yaw. Identity: `robot_cur_yaw + robot_add_yaw = original_raw_yaw`.
- **Example**: If raw yaw = 7.5 rad → k=1, temp=1.217, j=0, robot_cur_yaw=1.217,
  robot_add_yaw = 2π ≈ 6.283. Recovered: 1.217 + 6.283 = 7.5 ✓

### `phi_ctrl` (φ_ctrl)
- **What**: The NMPC solver's predicted optimal heading angle.
- **Source**: `predict_input(1)` from `localplanner->getNMPCPredictXU()`.
- **Frame**: Same wrapped coordinate system as `robot_cur_yaw` (since the NMPC
  state uses `robot_cur_yaw`). The "true" world heading = `phi_ctrl + robot_add_yaw`.
- **Semantics**: This is the direction the robot should travel (not necessarily
  where the gimbal points). The NMPC model is `ẋ = v·cos(θ), ẏ = v·sin(θ)`.

### `v_ctrl`
- **What**: The NMPC solver's predicted optimal scalar speed.
- **Source**: `predict_input(0)`.
- **Unit**: m/s. Can be negative in DISPENSE (anti-stuck) mode: `v_ctrl = −0.5 · v_ctrl`.

### `world_heading` (NEW — replaces old `delta`)
- **What**: The true world-frame heading angle.
- **Formula**: `phi_ctrl + robot_add_yaw`
- **Purpose**: Used to decompose the scalar speed v into world-frame vx/vy:
  ```
  vx = v · cos(world_heading)
  vy = v · sin(world_heading)
  ```

### OLD `delta` (REMOVED)
- **Was**: `(phi_ctrl + robot_add_yaw) − (robot_lidar_yaw + robot_add_yaw)`
  which simplified to `phi_ctrl − robot_lidar_yaw`.
- **Was**: The angle difference between the desired heading and the gimbal yaw.
- **Was used for**: Gimbal-frame decomposition (now removed). Since we decompose
  into world-frame vx/vy, this is no longer needed.

---

## 4. World-Frame Velocity Decomposition — Math

The NMPC outputs scalar speed `v` and desired heading `φ` (wrapped coordinates).
The tracking manager decomposes into world-frame (vx, vy):

```
world_heading = φ_ctrl + robot_add_yaw

vx_world = v · cos(world_heading)     (world X velocity)
vy_world = v · sin(world_heading)     (world Y velocity)
```

**Why this works**: The NMPC optimizes the trajectory `(x(t), y(t))` in the world
(map) frame. The kinematic model is `ẋ = v·cos(θ)`, `ẏ = v·sin(θ)`. The NMPC outputs
`(v, θ_desired)` — the speed and direction of travel. Decomposing along the world
heading directly gives world-frame velocities.

**Contrast with old gimbal-frame approach**: Previously, `delta = φ − θ_gimbal`
was used to project into the gimbal's local frame. This required the MCU to know
the gimbal orientation to convert back to world frame for motor control. Now the
MCU receives world-frame velocities directly.

---

## 5. USB CDC Protocol — Frameless (Raw Packed Structs)

USB CDC preserves message boundaries: each `CDC_Transmit_FS()` / `serial_.write()`
delivers a discrete packet. **No SOF/EOF/CRC framing is needed.** Both sides
simply `memcpy` the raw struct bytes.

### TX: NUC → MCU — NavigationFrame (8 bytes, raw)

Matches MCU-side `vision_rx_t` exactly:

```
 NUC struct             MCU struct
 ──────────             ──────────
 NavigationFrame        vision_rx_t
 ┌────────────┐         ┌────────────┐
 │ float vx   │  byte 0–3  → │ float v    │   world-frame vx [m/s]
 │ float vy   │  byte 4–7  → │ float w    │   world-frame vy [m/s]
 └────────────┘         └────────────┘
    8 bytes                8 bytes
```

MCU handler:
```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
  if (*Len >= sizeof(vision_rx_t)) {
    memcpy(&vision_rx, Buf, sizeof(vision_rx_t));  // 8 bytes
  }
}
```

**Removed from old protocol**: SOF (1B), CRC8 (1B), EOF (1B), z_angle (4B),
received (1B), arrived (1B) — total saved: 9 bytes. Frame reduced from 17 → 8 bytes.

### RX: MCU → NUC — MCUDataFrame (4 bytes, raw)

```
 Offset  Size   Type     Field        Encoding
 ──────  ────   ──────   ─────        ────────
  0      4      float    yaw_angle    IEEE 754 single [deg] gimbal yaw (world frame)
```

The NUC accumulates 4 bytes, then `memcpy`s into a float.
`mcu_communicator` converts **degrees → radians** (`yaw_rad = yaw_deg * π / 180`)
before publishing on `/mcu/yaw_angle` (in radians). This ensures all ROS
consumers (e.g. `setRPY()`) receive the value in the expected unit.
No SOF/EOF/CRC — USB CDC handles integrity.

**Removed from old protocol**: SOF, CRC8, EOF, all referee data, HP, enemy
positions, chassis_imu, etc. (89 → 4 bytes).

### MotionCommandFrame (unchanged, 7 bytes, SOF/CRC/EOF framed)

```
 Offset  Size   Type     Field
 ──────  ────   ──────   ─────
  0      1      uint8    sof          0x92
  1      1      uint8    motion_mode  (0=attack, 1=defend, 2=move, 3=brake)
  2      1      uint8    hp_up        (0/1)
  3      1      uint8    bullet_up    (0/1)
  4      1      uint8    bullet_num
  5      1      uint8    crc8
  6      1      uint8    eof          0xFE
```

Note: MotionCommandFrame still uses SOF/CRC/EOF framing. The MCU may handle
it separately from the vision_rx_t path (different length check in CDC_Receive_FS,
or via a different interface).

---

## 6. Topic Chain Summary

```
/sentry_des_speed   (sentry_msgs/slaver_speed)   tracking_node → hit_bridge
        │
        ▼
/cmd_vel            (geometry_msgs/Twist)         hit_bridge → mcu_communicator
        │
        ▼
USB CDC raw         (NavigationFrame, 8B)        mcu_communicator → MCU
```

```
MCU → USB CDC raw float (MCUDataFrame, 4B) → mcu_communicator → /mcu/yaw_angle
```

| Topic | Publisher | Subscriber | Message Type | Rate |
|-------|-----------|------------|--------------|------|
| `/sentry_des_speed` | `tracking_node` | `hit_bridge` | `sentry_msgs/slaver_speed` | ~100 Hz |
| `/cmd_vel` | `hit_bridge` | `mcu_communicator` | `geometry_msgs/Twist` | ~100 Hz |
| `/dstar_status` | `hit_bridge` | (unused) | `std_msgs/Bool` | ~100 Hz |
| `/mcu/yaw_angle` | `mcu_communicator` | (available) | `std_msgs/Float32` | MCU rate |
| USB CDC TX (nav) | `mcu_communicator` | MCU | NavigationFrame (8B raw) | 50 Hz |
| USB CDC TX (motion) | `mcu_communicator` | MCU | MotionCommandFrame (0x92, 7B) | on-demand |
| USB CDC RX | MCU | `mcu_communicator` | MCUDataFrame (4B raw float) | MCU rate |

---

## 7. slaver_speed.msg Field Mapping

The message field names are legacy and do NOT reflect their current usage:

| slaver_speed field | Original meaning | **Current actual value** | Unit |
|-------------------|------------------|--------------------------|------|
| `angle_target` | target yaw angle | **world-frame vx** | m/s |
| `angle_current` | current radar yaw | **world-frame vy** | m/s |
| `line_speed` | target linear speed | **unused (always 0.0)** | — |
| `xtl_flag` | motion mode | motion mode (0–3 voted) | enum |
| `in_bridge` | bridge crossing | bridge crossing flag | bool |

---

## 8. Special Modes

### 8.1 Arrival (stopped)
When `target_distance < 0.3m`, `arrival_goal = true`. All velocities are zeroed:
```cpp
MPC_Control = (0.0, 0.0, 0.0, checkMotionMode());
```
MCU receives `vx=0, vy=0`.

### 8.2 DISPENSE (escape/anti-stuck)
Speed is reversed at half magnitude before world-frame decomposition:
```cpp
v_ctrl = -0.5 * v_ctrl;   // before cos/sin decomposition
```
Since world_heading stays the same but v is negative, the resulting vx/vy
point in the opposite direction — the robot reverses along the same heading.

### 8.3 Solver failure
If `solveNMPC()` returns 0, `predict_input` stays at `Vector4d::Zero()`,
giving `v=0, φ=0` → `vx=0, vy=0`. The robot safely stops.

### 8.4 Gazebo simulation path
Uses the same world-frame decomposition but without yaw wrapping (Gazebo yaw
is always in [−π,π]). `phi_ctrl` IS the world heading directly.

---

## 9. Data Flow Diagram (per cycle)

```
                          ┌─────────────────────────┐
                          │   hdl_localization       │
                          │   /odom → (x,y,θ)       │
                          └───────────┬─────────────┘
                                      │ /odom_local (remapped)
                                      ▼
┌────────────────────────────────────────────────────────────────┐
│  tracking_node::rcvLidarIMUPosCallback()                       │
│                                                                │
│  1. Extract pose (x,y) and robot_lidar_yaw from quaternion     │
│  2. Wrap yaw → robot_cur_yaw + robot_add_yaw                  │
│  3. Build MPC state: (x, y, |v|, robot_cur_yaw)               │
│  4. solveNMPC(state) → predict_input = (v, φ_ctrl, a, ω)      │
│  5. world_heading = φ_ctrl + robot_add_yaw                     │
│  6. vx = v·cos(world_heading)  vy = v·sin(world_heading)      │
│  7. publishSentryOptimalSpeed( (vx, vy, 0, mode) )            │
│       → /sentry_des_speed                                      │
└──────────────────────────┬─────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────┐
│  hit_bridge::speedCallback()                 │
│  twist.linear.x = vx,  .y = vy              │
│  → /cmd_vel                                  │
│  → /dstar_status (arrived = vx&vy < 0.01?)  │
└──────────────────────────┬───────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────┐
│  mcu_communicator                            │
│  cmdVelCallback: store vx, vy                │
│  Timer @ 50Hz: sendNavigationCommand()       │
│    raw 8 bytes {float vx, float vy}          │
│    → USB CDC /dev/ttyUSB0 @ 115200           │
└──────────────────────────┬───────────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │   MCU (C板)   │
                    │  vx,vy → 麦轮  │
                    │              │
                    │  TX: yaw_angle│
                    └──────┬───────┘
                           │ USB CDC raw float (4B)
                           ▼
                    /mcu/yaw_angle
```

---

## 10. Changes from Previous Version

| Item | Before | After |
|------|--------|-------|
| Velocity frame | Gimbal (body) frame | **World (map) frame** |
| Decomposition | `δ = φ − gimbal_yaw` | `world_heading = φ + add_yaw` |
| z_angle output | `twist.angular.z = 0.0` | **Removed entirely** |
| NavFrame size | 17 bytes (SOF+vx+vy+z_angle+received+arrived+CRC+EOF) | **8 bytes raw (vx+vy) — matches MCU vision_rx_t** |
| MCU RX frame | 89 bytes (SOF+all referee/HP/enemy+CRC+EOF) | **4 bytes raw (float yaw_angle)** |
| Framing | SOF/CRC8/EOF on all frames | **None (raw packed structs via USB CDC)** |
| MCU topics published | ~30 (HP, scores, enemies, etc.) | **1** (`/mcu/yaw_angle`) |
| hit_bridge angular.z | Forwarded `line_speed` | Fixed `0.0` |

### Files changed:
1. `tracking_manager.cpp` — World-frame vx/vy decomposition (both LiDAR and Gazebo paths)
2. `hit_bridge.cpp` — Removed angular.z forwarding, updated comments
3. `mcu_comm.hpp` — Frameless NavigationFrame (8B raw) and MCUDataFrame (4B raw)
4. `mcu_communicator.cpp` — Raw write for nav TX, raw float accumulation for yaw RX, removed SOF/EOF/CRC parsing

---

## 11. Launch Configuration

From `HIT_intergration_test.launch`:

```xml
<!-- 10a. hit_bridge -->
<node pkg="tracking_node" type="hit_bridge" name="hit_bridge" output="screen">
    <param name="cmd_vel_topic"  value="/cmd_vel" />
    <param name="speed_topic"    value="/sentry_des_speed" />
    <param name="arrived_topic"  value="/dstar_status" />
</node>

<!-- 10b. mcu_communicator -->
<node pkg="decision_node" type="mcu_communicator" name="mcu_communicator" output="screen"
      if="$(arg enable_mcu_communicator)">
    <param name="serial_port" value="$(arg serial_port)" />
    <param name="baudrate"    value="$(arg serial_baudrate)" />
    <param name="nav_frequency" value="50.0" />
</node>
```

---

*End of report.*
