# Yaw Exchange Removal Report: Direct vx/vy in Gimbal Frame

## Summary

Eliminated the OMNI/OMEGA yaw exchange decomposition in `RM_Sentry_2026`. `tracking_manager` now computes gimbal-frame `vx`/`vy` directly using the **gimbal (LiDAR) yaw** (`robot_lidar_yaw`), and `hit_bridge` passes them straight through to `/cmd_vel`. MCU communication remains serial (`/dev/ttyUSB0`, 115200 baud).

---

## What Changed

### Old Velocity Flow (OMNI mode)

```
tracking_manager → slaver_speed(heading, chassis_yaw, v, mode)
  → hit_bridge (δ = heading − chassis_yaw, vx = v·cos(δ), vy = v·sin(δ))
  → cmd_vel(Twist) → mcu_communicator (serial)
```

### New Velocity Flow

```
tracking_manager (δ = heading − gimbal_yaw, vx = v·cos(δ), vy = v·sin(δ))
  → slaver_speed(vx, vy, z_angle=0, mode)
  → hit_bridge (direct passthrough)
  → cmd_vel(Twist) → mcu_communicator (serial, unchanged)
```

Key difference: decomposition now uses **gimbal yaw** (`robot_lidar_yaw`) instead of chassis yaw (`robot_cur_yaw`), and happens in `tracking_manager` instead of `hit_bridge`.

---

## Files Modified

### tracking_manager.cpp
- Removed `use_omega_output_` param loading and WARN message
- Replaced OMNI/OMEGA mode switch with direct gimbal-frame vx/vy computation:
  ```cpp
  double delta = (phi_ctrl + robot_add_yaw) - (robot_lidar_yaw + robot_add_yaw);
  MPC_Control(0) = v_ctrl * std::cos(delta);  // gimbal-frame vx
  MPC_Control(1) = v_ctrl * std::sin(delta);  // gimbal-frame vy
  MPC_Control(2) = 0.0;                       // z_angle
  ```
- Simplified stopped-state case to all zeros

### hit_bridge.cpp
- Removed `g_use_omega_output` flag and all OMNI/OMEGA mode logic
- Now pure passthrough:
  - `twist.linear.x = msg->angle_target` (vx)
  - `twist.linear.y = msg->angle_current` (vy)
  - `twist.angular.z = msg->line_speed` (z_angle)
- Arrival detection: checks all three fields near zero

### trajectory_planning.launch
- Removed `use_omega_output` arg and param

### HIT_intergration_test.launch
- Removed `use_omega_output` arg declaration and usage

---

## slaver_speed Field Mapping

| Field | Old (OMNI) | New |
|---|---|---|
| `angle_target` | desired heading (rad) | gimbal-frame vx (m/s) |
| `angle_current` | chassis yaw (rad) | gimbal-frame vy (m/s) |
| `line_speed` | linear speed v (m/s) | angular velocity z (rad/s), currently 0 |
| `xtl_flag` | motion mode | motion mode (unchanged) |
| `in_bridge` | bridge flag | bridge flag (unchanged) |

## Build Results

| Workspace | Status |
|-----------|--------|
| `RM_Sentry_2026/DecisionNode` | ✅ Built (serial `mcu_communicator`) |
| `RM_Sentry_2026/HIT_code/sentry_planning_ws` | ✅ Built (`hit_bridge` + `tracking_node`) |
