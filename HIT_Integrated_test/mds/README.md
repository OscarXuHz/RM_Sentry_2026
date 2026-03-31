# HIT Integrated Test — Documentation Index

All fix reports, problem reports, and reference documentation for the HIT sentry navigation stack.

---

## Fix Reports (chronological)

| Report | Summary |
|--------|---------|
| [NUC_ALIGNMENT_REPORT.md](NUC_ALIGNMENT_REPORT.md) | NUC compute module physical/network alignment |
| [MAP_GRADIENT_FIX_REPORT.md](MAP_GRADIENT_FIX_REPORT.md) | PCD map floor gradient normalization (z-height correction) |
| [Z_FILTER_FIX_REPORT.md](Z_FILTER_FIX_REPORT.md) | z_ceiling / floor detection parameter tuning |
| [LOCALIZATION_FIX_REPORT.md](LOCALIZATION_FIX_REPORT.md) | Globalmap PCD swap, init pose to origin, 90° offset fix |
| [OBSTACLE_AVOIDANCE_FIX_REPORT.md](OBSTACLE_AVOIDANCE_FIX_REPORT.md) | ⚠️ HISTORICAL — A* planner + MPC obstacle avoidance (old dual-LiDAR) |
| [YAW_EXCHANGE_REMOVAL_REPORT.md](YAW_EXCHANGE_REMOVAL_REPORT.md) | Removed yaw exchange logic causing heading errors |
| [UPSIDE_DOWN_POINTCLOUD_FIX_REPORT.md](UPSIDE_DOWN_POINTCLOUD_FIX_REPORT.md) | ⚠️ HISTORICAL — livox_cloudpoints_processor Y/Z negation (ws_cloud path not active) |
| [FIX44_DIAGNOSTIC_REPORT.md](FIX44_DIAGNOSTIC_REPORT.md) | ⚠️ HISTORICAL — Fix 44 diagnostic: path divergence, obstacle avoidance |

## Problem Reports

| Report | Summary | Status |
|--------|---------|--------|
| [PROBLEM_REPORT_LIDAR_TOPICS.md](PROBLEM_REPORT_LIDAR_TOPICS.md) | ⚠️ HISTORICAL — LiDAR topic mismatch (old two-lidar setup) | Superseded |
| [PROBLEM_REPORT_LOCALIZATION_TILT.md](PROBLEM_REPORT_LOCALIZATION_TILT.md) | ⚠️ HISTORICAL — Localization tilt from dual-IMU mismatch | ✅ Fixed |
| [PROBLEM_REPORT_CMD_VEL_FLUCTUATION.md](PROBLEM_REPORT_CMD_VEL_FLUCTUATION.md) | ⚠️ HISTORICAL — cmd_vel oscillation (old dual-LiDAR) | ✅ Fixed |

## System Reference

| Report | Summary |
|--------|---------|
| [SYSTEM_ARCHITECTURE_REPORT.md](SYSTEM_ARCHITECTURE_REPORT.md) | **Master reference** — Full SLAM + navigation architecture (16 sections) |
| [HIT_LIDAR_PIPELINE_REPORT.md](HIT_LIDAR_PIPELINE_REPORT.md) | Complete HIT lidar data pipeline: driver → localization/planning |
| [HIT_COORDINATE_SYSTEMS_REPORT.md](HIT_COORDINATE_SYSTEMS_REPORT.md) | HIT TF tree, coordinate transforms, frame relationships |
| [HIT_MCU_COMMAND_PIPELINE_REPORT.md](HIT_MCU_COMMAND_PIPELINE_REPORT.md) | MCU command chain: MPC output → serial → chassis |
| [PARAMETER_TUNING_GUIDE.md](PARAMETER_TUNING_GUIDE.md) | Key parameters across all nodes with recommended values |
| [OLD_NAV_LIDAR_PIPELINE_REPORT.md](OLD_NAV_LIDAR_PIPELINE_REPORT.md) | ⚠️ HISTORICAL — Old_nav lidar pipeline (reference only) |
| [OLD_NAV_COORDINATE_SYSTEMS_REPORT.md](OLD_NAV_COORDINATE_SYSTEMS_REPORT.md) | ⚠️ HISTORICAL — Old_nav TF tree (reference only) |

## Planning Stack Documentation

| Report | Summary |
|--------|---------|
| [TOPO_GUARD_CONNECTION_POINTS.md](TOPO_GUARD_CONNECTION_POINTS.md) | **NEW** — Guard & connection point mechanics in TopoSearch |
| [DISPENSE_MODE_REPORT.md](DISPENSE_MODE_REPORT.md) | **NEW** — DISPENSE (escape) mode: trigger, behavior, exit |
| [PLAN_1_MINCO_SPACETIME.md](PLAN_1_MINCO_SPACETIME.md) | MINCO spacetime trajectory optimization theory |
| [PLAN_2_SFC_TRAJECTORY_GEN.md](PLAN_2_SFC_TRAJECTORY_GEN.md) | Safe Flight Corridor trajectory generation |
| [PLAN_3_CERES_SOLVER.md](PLAN_3_CERES_SOLVER.md) | Ceres solver integration for trajectory optimization |

## History & Sync

| Report | Summary |
|--------|---------|
| [PLANNING_FIX_HISTORY.md](PLANNING_FIX_HISTORY.md) | Complete chronological log of all planning fixes (Fix 1–48) |
| [REPO_SYNC_REPORT.md](REPO_SYNC_REPORT.md) | Repository sync status between Old_nav, HIT, and upstream |
