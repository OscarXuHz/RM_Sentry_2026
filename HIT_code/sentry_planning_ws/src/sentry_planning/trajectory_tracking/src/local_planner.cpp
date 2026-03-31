//
// Created by hitcrt on 2023/4/5.
//

#include "../include/local_planner.h"
#include <math.h>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>

const int LocalPlanner::n;
const int LocalPlanner::m;

void LocalPlanner::linearation(double time, double robot_cur_yaw)
{
    // MPC参考轨迹，速度，时间，预测轨迹和输入全部清空
    ref_phi.clear();
    ref_speed.clear();
    ref_phi.push_back(robot_cur_yaw);

    observation.time = time;
    ref_trajectory.clear();
    ref_velocity.clear();
    reference_time.clear();
    // Fix 39a: Save previous predicted trajectory BEFORE clearing, so we can
    // search for obstacles around it.  The reference trajectory alone misses
    // obstacles that lie near the actual predicted (diverged) path.
    prev_predictState_ = predictState;
    predictState.clear();
    predictInput.clear();
    obs_points.clear();
    obs_points_t.clear();
    obs_num = 0;
    double sum_time = accumulate(ref_time.begin(), ref_time.end(), 0.0);

    int segment_index;
    double total_time;  // 该段前的总时间
    getTrackingSegmentIndex(time, segment_index, total_time);
    double segment_time = time - total_time;
    bool unknown_obs_exist = false;
    std::vector<bool> occ_flag;  // 记录参考点中哪个点在障碍物中

    bool evade_flag = false;
    if(motion_mode == 8){  // 防守进攻模式下
        if((global_map->odom_position.head(2) - target_point.head(2)).norm() < 1.2){
            evade_flag = true;
        }else{
            evade_flag = false;
        }
    }

    if(!evade_flag) {
        for (int i = 0; i < 20; i++) {  /// 获取参考轨迹与障碍物处理，如果在障碍物里直接进行处理
            if (segment_index < reference_velocity.size() - 1) //根据当前的时间判断是不是还在全局参考的index内
            {
                Eigen::Vector3d velocity_temp;
                Eigen::Vector3d position_temp;

                velocity_temp.x() = reference_velocity[segment_index].x() +
                                    (reference_velocity[segment_index + 1].x() -
                                     reference_velocity[segment_index].x()) * segment_time / dt;
                velocity_temp.y() = reference_velocity[segment_index].y() +
                                    (reference_velocity[segment_index + 1].y() -
                                     reference_velocity[segment_index].y()) * segment_time / dt;
                velocity_temp.z() = 0.0;

                position_temp.x() = reference_path[segment_index].x() +
                                    (reference_path[segment_index + 1].x() - reference_path[segment_index].x()) *
                                    segment_time / dt;
                position_temp.y() = reference_path[segment_index].y() +
                                    (reference_path[segment_index + 1].y() - reference_path[segment_index].y()) *
                                    segment_time / dt;
                position_temp.z() = 0.0;

                ref_trajectory.push_back(position_temp);
                ref_velocity.push_back(velocity_temp);
                reference_time.push_back(time + i * dt);
                segment_index++;
            } else {  // Past trajectory end — hold final position, command ZERO velocity
                // (Fix 20) Use v=0 at goal so MPC actively decelerates instead of
                // matching the last segment's cruise speed.
                ref_trajectory.push_back(reference_path[reference_path.size() - 1]);
                Eigen::Vector3d velocity_temp = Eigen::Vector3d::Zero(3);
                ref_velocity.push_back(velocity_temp);
                reference_time.push_back(time + i * dt);
            }

            Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
//        std::vector<Eigen::Vector3d> obs_point_temp;
//        std::vector<std::pair<int, Eigen::Vector3d>> obs_point_temp_t;
            if (!global_map->isOccupied(pos_idx.x(), pos_idx.y(), pos_idx.z())) {
                occ_flag.push_back(false);
//            obs_points_t.push_back(obs_point_temp_t);
            } else {
                occ_flag.push_back(true);
            }
        }
    }else{
        getCircleEvadeTraj(global_map->odom_position, ref_trajectory, ref_velocity, reference_time);
        for (int i = 0; i < 20; i++) {
            Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
            if (!global_map->isOccupied(pos_idx.x(), pos_idx.y(), pos_idx.z())) {
                occ_flag.push_back(false);
                std_msgs::Bool decision_flag;
                decision_flag.data = true;
                redecision_pub.publish(decision_flag);
            } else {
                std_msgs::Bool decision_flag;
                decision_flag.data = true;
                redecision_pub.publish(decision_flag);
                occ_flag.push_back(true);
            }
        }

        start_tracking_time = ros::Time::now();
    }

    // ── Obstacle search for each horizon step ──────────────────────────
    // CRITICAL FIX: The old code used a `safe_constraint` gate that only
    // searched for obstacles when the reference point itself (or ±3
    // neighbours) was inside an occupied cell.  This created a binary
    // toggle: when an obstacle was near the edge of the path, one frame
    // had 0 constraints, the next had hundreds → massive QP structure
    // flip → solver oscillated between two solutions → visible path
    // flicker.
    //
    // Fix: ALWAYS search for obstacles around each reference point.
    // Cap to the MAX_OBS_PER_STEP closest obstacles per step so the QP
    // structure stays approximately constant across frames.
    //
    // Fix 39a: Also search around the PREVIOUS predicted trajectory.
    // The MPC predicted path can diverge significantly from the reference
    // when avoiding obstacles.  Searching only around ref_trajectory meant
    // obstacles near the actual predicted path were invisible → path went
    // straight through walls/obstacles.
    // Fix 39b: Wider search radius and more obstacles per step for better
    // wall representation.  With only 8 points the solver could squeeze
    // between discrete obstacle points on a continuous wall.
    // Fix 42a: Reduced from 30→20 (±1.0m).  Obstacles beyond 1.0m have
    // negligible barrier influence (sqrt(0.25+0.6)=0.92m for static) and
    // only bloat the QP.  Combined with fewer sectors, this cuts total
    // obs from ~219 to ~60-80, letting the SQP actually converge.
    static const int OBS_SEARCH_HALF = 20;
    static const size_t MAX_OBS_PER_STEP = 6;     // Fix 42a: 12→6 sectors
    for(int i = 0; i < 20; i++){
        Eigen::Vector3i pos_idx = global_map->coord2gridIndex(ref_trajectory[i]);
        std::vector<Eigen::Vector3d> obs_point_temp;
        std::vector<std::pair<int, Eigen::Vector3d>> obs_point_temp_t;

        // Fix 43b: Removed exist_second_height gate.  Previously, when the
        // reference point was in a "bridge area" (exist_second_height=true),
        // ALL obstacle search was skipped → zero constraints → MPC completely
        // blind → path went straight through obstacles in those areas.
        // Now obstacles are always searched regardless of bridge/height status.
        {
            // Temporary storage with distance for sorting
            struct ObsEntry {
                double dist2;
                int type;
                Eigen::Vector3d pos;
            };
            std::vector<ObsEntry> candidates;

            // Helper: search a window around a center point and add obstacles
            // Fix 55b: Correct obstacle type classification.
            // Previously, ANY cell with l_data>0 got type=1 (dynamic, 0.4m clearance),
            // including static walls visible to lidar. In corridors, BOTH walls get
            // type=1 → MPC needs 0.4m clearance from each → not enough space → barrier
            // overwhelms tracking cost → negative v_ctrl → robot stuck.
            // Now: if a lidar cell is near a static map entry (±STATIC_NBR cells),
            // classify as type=0 (static, 0.2m clearance). Only genuinely dynamic
            // obstacles (no nearby static entry) get type=1.
            static const int STATIC_NBR = 6;  // Fix 60: ±6 cells = ±0.3m (was 4) for localization drift
            auto searchWindow = [&](const Eigen::Vector3i& center_idx) {
                for(int di = -OBS_SEARCH_HALF; di <= OBS_SEARCH_HALF; di++){
                    for(int dj = -OBS_SEARCH_HALF; dj <= OBS_SEARCH_HALF; dj++){
                        int nx = center_idx.x() + di;
                        int ny = center_idx.y() + dj;
                        if(global_map->isOccupied(nx, ny, center_idx.z())){
                            Eigen::Vector3i temp_idx = {nx, ny, center_idx.z()};
                            Eigen::Vector3d temp_pos = global_map->gridIndex2coord(temp_idx);
                            double dx = ref_trajectory[i].x() - temp_pos.x();
                            double dy = ref_trajectory[i].y() - temp_pos.y();
                            int type = 0;  // default: static
                            if (global_map->isLocalOccupied(nx, ny, center_idx.z())) {
                                // Lidar sees something here. Is it a known wall?
                                if (global_map->data[nx * global_map->GLY_SIZE + ny] != 1) {
                                    // Fix 61b: Use precomputed dilated static map instead of O(169) inner loop
                                    if (nx >= 0 && nx < global_map->GLX_SIZE &&
                                        ny >= 0 && ny < global_map->GLY_SIZE &&
                                        !global_map->near_static_data[nx * global_map->GLY_SIZE + ny]) {
                                        type = 1;  // genuinely dynamic
                                    }
                                }
                                // else: exact cell in static map → type stays 0
                            }
                            candidates.push_back({dx*dx + dy*dy, type, temp_pos});
                            if(type == 1) unknown_obs_exist = true;
                        }
                    }
                }
            };

            // Search around reference trajectory point
            searchWindow(pos_idx);

            // Fix 42a-rev2: RE-ENABLED prev_predictState_ search.
            // The predicted path can deviate 2+ meters from the reference
            // (maxDev=2.87m observed).  Searching only around the reference
            // misses obstacles near the actual predicted path → the path
            // goes through those obstacles → checkfeasible() fires "not safe"
            // every frame → perpetual replan loop.
            //
            // With sector-based selection, re-enabling this search is safe:
            // a sector can only hold ONE obstacle (the closest), so
            // candidates from both searches compete within sectors and
            // the total output is still bounded at N_SECTORS per step.
            if (i < (int)prev_predictState_.size()
                && (std::abs(prev_predictState_[i](0)) > 0.1 || std::abs(prev_predictState_[i](1)) > 0.1)) {
                Eigen::Vector3d prev_pred_pos(prev_predictState_[i](0),
                                              prev_predictState_[i](1), 0.0);
                double ref_pred_dist = (ref_trajectory[i].head<2>() - prev_pred_pos.head<2>()).norm();
                if (ref_pred_dist > 0.3) {
                    Eigen::Vector3i pred_idx = global_map->coord2gridIndex(prev_pred_pos);
                    searchWindow(pred_idx);
                }
            }

            // Fix 43c: Also search around the robot's CURRENT position for
            // the first few MPC steps.  Even with reduced MAX_LEAD_TIME
            // (Fix 43a), this belt-and-suspenders ensures obstacles right
            // next to the robot are always visible to the MPC barrier.
            if (i < 5) {
                Eigen::Vector3i robot_idx = global_map->coord2gridIndex(global_map->odom_position);
                double robot_ref_dist = (ref_trajectory[i].head<2>() - global_map->odom_position.head<2>()).norm();
                if (robot_ref_dist > 0.2) {
                    searchWindow(robot_idx);
                }
            }

            // Deduplicate candidates (same grid cell can appear from both/all searches)
            {
                std::sort(candidates.begin(), candidates.end(),
                          [](const ObsEntry& a, const ObsEntry& b){
                              if (a.pos.x() != b.pos.x()) return a.pos.x() < b.pos.x();
                              return a.pos.y() < b.pos.y();
                          });
                auto it = std::unique(candidates.begin(), candidates.end(),
                                      [](const ObsEntry& a, const ObsEntry& b){
                                          return std::abs(a.pos.x() - b.pos.x()) < 1e-6
                                              && std::abs(a.pos.y() - b.pos.y()) < 1e-6;
                                      });
                candidates.erase(it, candidates.end());
                // Recompute distances to ref_trajectory after dedup
                for (auto& c : candidates) {
                    double dx = ref_trajectory[i].x() - c.pos.x();
                    double dy = ref_trajectory[i].y() - c.pos.y();
                    c.dist2 = dx*dx + dy*dy;
                }
            }

            // Fix 42a: Angular-sector selection with 8 sectors + distance filter.
            // Fix 41a used 12 sectors → up to 12 obs/step → 240 total → SQP choked.
            // 8 sectors (45° each) gives good angular coverage (left/right/front/rear
            // + diagonals) while keeping total obs manageable (~100-120).
            // Max distance 1.0m filter removes far-away obstacles that contribute
            // negligible barrier gradient but bloat the QP.
            {
                static const int N_SECTORS = 8;
                static const double MAX_OBS_DIST2 = 1.0 * 1.0;  // 1.0m max
                struct SectorBest {
                    double dist2;
                    int type;
                    Eigen::Vector3d pos;
                    bool valid;
                };
                SectorBest sectors[8];
                for (int s = 0; s < N_SECTORS; s++) {
                    sectors[s] = {1e18, 0, Eigen::Vector3d::Zero(), false};
                }

                for (auto& c : candidates) {
                    if (c.dist2 > MAX_OBS_DIST2) continue;  // skip far obstacles
                    double dx = c.pos.x() - ref_trajectory[i].x();
                    double dy = c.pos.y() - ref_trajectory[i].y();
                    double angle = std::atan2(dy, dx) + M_PI;  // [0, 2π)
                    int sector = std::min((int)(angle / (2.0 * M_PI) * N_SECTORS), N_SECTORS - 1);
                    if (!sectors[sector].valid || c.dist2 < sectors[sector].dist2) {
                        sectors[sector] = {c.dist2, c.type, c.pos, true};
                    }
                }

                for (int s = 0; s < N_SECTORS; s++) {
                    if (sectors[s].valid) {
                        obs_point_temp.push_back(sectors[s].pos);
                        obs_point_temp_t.push_back(std::make_pair(sectors[s].type, sectors[s].pos));
                    }
                }
            }
        }

        obs_points_t.push_back(obs_point_temp_t);
        obs_points.push_back(obs_point_temp);
        obs_num += obs_point_temp.size();
    }

    // NOTE: The original code cleared ALL obstacle constraints (including static
    // walls) when no dynamic obstacles were nearby.  This caused the MPC to be
    // completely blind to walls, allowing the robot to crash into them.
    // Static obstacle constraints are now always kept active so the MPC can
    // push the robot away from walls even when no point-cloud obstacles exist.
    // Dynamic obstacles (type 1) still receive a tighter penalty in the solver
    // via the type tag on each obs_point.
    (void)unknown_obs_exist;  // retained for future per-type penalty tuning
    // Update obstacle data via shared pointer - the solver's internal constraint
    // sees these updates through the shared_ptr without needing to recreate.
    mpcInterface_.obsConstraintPtr_->timeTrajectory_ = reference_time;
    mpcInterface_.obsConstraintPtr_->obs_points_t_ = obs_points_t;

    // Create solver ONCE on first call.  Recreating every cycle caused SIGSEGV:
    // OCS2 SqpMpc uses nThreads=2; if the solver fails mid-solve, its threads
    // may still be running when reset() destroys the object -> use-after-free.
    if (!mpcSolverPtr_) {
        // Fix 46f: μ=20, δ=0.5. Fix 44b's μ=40 δ=1.0 was too strong for
        //   narrow corridors — even at 0.3m from an obstacle, barrier
        //   penalty dominated Q=150 tracking cost → v_ctrl = 0.
        //   With μ=20 δ=0.5 and reduced thresholds (0.04/0.16),
        //   the barrier only activates close to obstacles and doesn't
        //   overpower position tracking.
        ocs2::RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(20.0, 0.5);
        stateCollisionSoftConstraintPtr = std::make_unique<ocs2::StateSoftConstraint>(std::make_unique<SentryCollisionConstraint>(mpcInterface_.obsConstraintPtr_), std::make_unique<ocs2::RelaxedBarrierPenalty>(barriercollisionPenaltyConfig));
        mpcInterface_.problem_.stateSoftConstraintPtr->add("stateCollisionBounds", std::move(stateCollisionSoftConstraintPtr));

        // ── HPIPM tuning ──────────────────────────────────────────────
        // The OCS2 loader does NOT read HPIPM settings from task.info.
        // Defaults are: reg_prim=1e-12 (≈0), mode=SPEED, ric_alg=0.
        // With near-zero regularization and SPEED mode, the Riccati
        // recursion fails on mildly ill-conditioned QPs.
        auto& hSet = mpcInterface_.sqpSettings().hpipmSettings;
        hSet.hpipmMode   = hpipm_mode::ROBUST;  // was SPEED
        hSet.reg_prim    = 1e-8;                 // was 1e-12
        mpcInterface_.sqpSettings().printSolverStatus = false;
        ROS_INFO("[LocalPlanner] HPIPM: mode=ROBUST, reg_prim=1e-8");
        ROS_INFO("[LocalPlanner] Fix 46f: barrier mu=20 delta=0.5, distStatic=0.04, distDynamic=0.16");

        mpcSolverPtr_.reset(new ocs2::SqpMpc(mpcInterface_.mpcSettings(), mpcInterface_.sqpSettings(), mpcInterface_.getOptimalControlProblem(), mpcInterface_.getInitializer()));
        ROS_INFO("[LocalPlanner] MPC solver created (one-time init)");
    }
}

void LocalPlanner::getFightTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw)
{
    if(reference_path.size() < 2){
        return;
    }

    // Fix 43a: Reduced MAX_LEAD_TIME from 2.0→0.5s.
    // THE SMOKING GUN: With MAX_LEAD_TIME=2.0, the MPC reference started
    // 2-3 meters ahead of the robot.  The obstacle search (±1.0m around
    // reference) missed obstacles near the robot entirely → predicted path
    // cut straight through walls/obstacles to reach the far-ahead reference.
    // With 0.5s, the reference is only ~0.3-0.5m ahead → obstacles near
    // the robot are within the search window and visible to the MPC barrier.
    // (Fix 42d's min() speed reduction makes the old speed plateau
    //  with MAX_LEAD_TIME=1.0 no longer a concern.)
    static const double MAX_LEAD_TIME = 0.5;  // seconds — was 2.0 (Fix 38c)
    double min_dist = 1e9;
    int closest_id = 0;
    for (int i = 0; i < (int)reference_path.size(); i++) {
        double d = std::hypot(state(0) - reference_path[i](0),
                              state(1) - reference_path[i](1));
        if (d < min_dist) { min_dist = d; closest_id = i; }
    }
    double max_time = closest_id * dt + MAX_LEAD_TIME;
    if (time > max_time) {
        start_tracking_time += ros::Duration(time - max_time);
        time = max_time;
    }

    /// 获得当前的MPC的参考轨迹
    linearation(time, robot_cur_yaw);

    // Fix 38a: Off-course deviation detection. When the robot drifts more
    // than OFF_COURSE_DIST from the nearest reference point, flag for
    // replanning. Previously always 0 → num_tracking_low > 4 never fired.
    static const double OFF_COURSE_DIST = 0.8;  // meters
    tracking_low_check_flag.insert(tracking_low_check_flag.begin(),
                                   min_dist > OFF_COURSE_DIST ? 1.0 : 0.0);
    if(tracking_low_check_flag.size() > 10){
        tracking_low_check_flag.pop_back();
    }

    double phi;
    double last_phi;

    // (Fix 27) Compute distance from each reference point to the goal
    // for the deceleration ramp. Applied to ref_speed only (not ref_velocity)
    // to keep position/velocity references consistent.
    Eigen::Vector3d goal_pos = reference_path[reference_path.size() - 1];

    for(int i = 0; i<planning_horizon; i++)  /// 这里我们更改只进行参考角度的更改
    {
        phi = atan2(ref_velocity[i](1),  ref_velocity[i](0));
        if(i == 0)  // 初始
        {
            if (phi - ref_phi[i] > M_PI) {
                phi = phi - 2 * M_PI;
            }
            if (phi - ref_phi[i] < -M_PI) {  // phi这里才是参考轨迹的yaw角
                phi = phi + 2 * M_PI;
            }
                // Fix 34b: Hysteresis for speed_direction to prevent frame-to-frame
                // flip-flopping at the ±π/2 boundary. Use different thresholds for
                // entering vs exiting reverse mode.
                // Fix 57: DISABLED — Robot is OMNIDIRECTIONAL.  speed_direction=-1
                // made ref_speed negative, confusing the MPC: the solver tried to
                // produce negative v_ctrl to match, which was then clamped to 0
                // (Fix 45c) → robot frozen.  With Fix 56 (v_ctrl flip instead of
                // clamp), negative ref_speed is less harmful but still unnecessary.
                // An omni robot can move in any direction at any heading, so there
                // is no concept of "reverse."
                // Always keep speed_direction = 1.
            {
                speed_direction = 1;
            }
            // Fix 57: speed_direction is always 1 now (omni robot).
            // No reverse mode needed — always use velocity direction as heading.
            {
                ref_phi[i] = phi;
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }else{
            if (phi - last_phi > M_PI) {  /// 两个参考轨迹点之间差pi
                phi -=  2 * M_PI;
            } else if (phi - last_phi < -M_PI) {
                phi +=  2 * M_PI;
            }

            if (ref_phi[i - 1] - phi > M_PI_2) {   /// 平滑处理
                ref_phi.push_back(phi + M_PI);
            } else if (ref_phi[i - 1] - phi < -M_PI_2) {
                ref_phi.push_back(phi - M_PI);
            } else{
                ref_phi.push_back(phi);
            }
            // Fix 57: speed_direction always 1  
            ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                     pow(ref_velocity[i](0), 2)));
        }
        last_phi = phi;

        // (Fix 27) Deceleration ramp: scale ref_speed near goal.
        // Applied here (not in linearation) so ref_trajectory positions
        // stay consistent with the polynomial while only the scalar speed
        // reference is reduced.  This fixes reference/predicted divergence.
        {
            double dist_to_goal = (ref_trajectory[i].head<2>() - goal_pos.head<2>()).norm();
            static const double DECEL_RAMP_DIST = 1.5;
            if (dist_to_goal < DECEL_RAMP_DIST) {
                double scale = std::max(0.05, dist_to_goal / DECEL_RAMP_DIST);
                ref_speed.back() *= scale;
            }
        }
    }

    // Fix 42d: Obstacle-proximity speed reduction (fixes double-multiplication bug).
    // Fix 39e's two-pass approach multiplied step_scale × horizon_scale,
    // causing effective_scale = 0.15² = 0.0225 → near-zero reference speed
    // → robot overshot reference by 2+ meters → "violent divergence."
    //
    // Fix: Use min(step_scale, horizon_scale) instead of multiplication.
    // This applies the TIGHTER of the two reductions, not both.
    // Fix 58: Reduced OBS_MIN_SCALE 0.25→0.15 since Fix 56 removes the
    // v_ctrl clamp that was the real cause of move-stop.  Stronger braking
    // near obstacles is now safe because the robot won't just freeze.
    {
        static const double OBS_SLOW_DIST = 1.5;   // start braking at 1.5m
        static const double OBS_MIN_SCALE = 0.15;   // Fix 58: 0.25→0.15; stronger braking near obstacles

        // Pass 1: find tightest obstacle across entire horizon
        double min_horizon_obs_dist = 1e9;
        for (int i = 0; i < (int)ref_speed.size() && i < (int)obs_points.size(); i++) {
            for (const auto& obs : obs_points[i]) {
                double d = (ref_trajectory[i].head<2>() - obs.head<2>()).norm();
                if (d < min_horizon_obs_dist) min_horizon_obs_dist = d;
            }
        }

        // Horizon-wide scale (look-ahead)
        double horizon_scale = 1.0;
        if (min_horizon_obs_dist < OBS_SLOW_DIST) {
            horizon_scale = OBS_MIN_SCALE
                          + (1.0 - OBS_MIN_SCALE) * (min_horizon_obs_dist / OBS_SLOW_DIST);
        }

        // Pass 2: per-step, use min(step_scale, horizon_scale) — NOT multiplication
        for (int i = 0; i < (int)ref_speed.size() && i < (int)obs_points.size(); i++) {
            double step_scale = 1.0;
            if (!obs_points[i].empty()) {
                double min_obs_dist = 1e9;
                for (const auto& obs : obs_points[i]) {
                    double d = (ref_trajectory[i].head<2>() - obs.head<2>()).norm();
                    if (d < min_obs_dist) min_obs_dist = d;
                }
                if (min_obs_dist < OBS_SLOW_DIST) {
                    step_scale = OBS_MIN_SCALE
                              + (1.0 - OBS_MIN_SCALE) * (min_obs_dist / OBS_SLOW_DIST);
                }
            }
            // Use the tighter reduction, not both
            double effective_scale = std::min(step_scale, horizon_scale);
            ref_speed[i] *= effective_scale;
        }
    }

    // Fix 58b: Curvature-based speed reduction.
    // When the reference path has sharp turns, reduce speed to prevent
    // overshooting the turn.  This fixes the "can't stop for turns" issue.
    {
        static const double TURN_SLOW_RATE = 0.5;  // radians/step threshold for braking
        static const double TURN_MIN_SCALE = 0.3;  // minimum speed fraction at sharp turns
        for (int i = 1; i < (int)ref_phi.size(); i++) {
            double dphi = std::abs(std::remainder(ref_phi[i] - ref_phi[i-1], 2.0 * M_PI));
            if (dphi > TURN_SLOW_RATE) {
                double turn_scale = TURN_MIN_SCALE
                    + (1.0 - TURN_MIN_SCALE) * std::max(0.0, 1.0 - (dphi - TURN_SLOW_RATE) / (M_PI - TURN_SLOW_RATE));
                ref_speed[i] *= turn_scale;
            }
        }
    }

}

void LocalPlanner::getTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw)
{
    if(reference_path.size() < 2){
        return;
    }

    // Fix 43a: MAX_LEAD_TIME 2.0→0.5 (see getFightTrackingTraj for rationale).
    static const double MAX_LEAD_TIME = 0.5;
    double min_dist = 1e9;
    int closest_id = 0;
    for (int i = 0; i < (int)reference_path.size(); i++) {
        double d = std::hypot(state(0) - reference_path[i](0),
                              state(1) - reference_path[i](1));
        if (d < min_dist) { min_dist = d; closest_id = i; }
    }
    double max_time = closest_id * dt + MAX_LEAD_TIME;
    if (time > max_time) {
        start_tracking_time += ros::Duration(time - max_time);
        time = max_time;
    }

    /// 获得当前的MPC的参考轨迹
    linearation(time, robot_cur_yaw);

    // Fix 38a: Off-course deviation detection (same as getFightTrackingTraj).
    static const double OFF_COURSE_DIST = 0.8;
    tracking_low_check_flag.insert(tracking_low_check_flag.begin(),
                                   min_dist > OFF_COURSE_DIST ? 1.0 : 0.0);
    if(tracking_low_check_flag.size() > 10){
        tracking_low_check_flag.pop_back();
    }

    double phi;
    double last_phi;

    for(int i = 0; i<planning_horizon; i++)  /// 这里我们更改只进行参考角度的更改
    {
        phi = atan2(ref_velocity[i](1), ref_velocity[i](0));
        if(i == 0)  // 初始
        {
            // 参考路径与速度
            if (ref_phi[i] - phi > M_PI) {  // 参考角度与实际状态反馈的角度差pi
                ref_phi[i] = (phi + 2*M_PI);  /// 相差一个周期
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else if (ref_phi[i] - phi < -M_PI) {
                ref_phi[i] = (phi - 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }else {  /// 小角度转向直接set
                ref_phi[i] = (phi);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }else{
            if (phi - last_phi > M_PI) {  /// 两个参考轨迹点之间差pi
                phi -=  2 * M_PI;
            } else if (phi - last_phi < -M_PI) {
                phi +=  2 * M_PI;
            }

            if (ref_phi[i - 1] - phi > M_PI) {   /// 平滑处理
                ref_phi.push_back(phi + 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else if (ref_phi[i - 1] - phi < -M_PI) {
                ref_phi.push_back(phi - 2 * M_PI);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            } else{
                ref_phi.push_back(phi);
                ref_speed.push_back(sqrt(pow(ref_velocity[i](1), 2) +
                                         pow(ref_velocity[i](0), 2)));
            }
        }
        last_phi = phi;
    }

    // Fix 42d: Obstacle-proximity speed reduction (same fix as getFightTrackingTraj)
    {
        static const double OBS_SLOW_DIST = 1.5;
        static const double OBS_MIN_SCALE = 0.25;   // Fix 42d: 0.15→0.25

        double min_horizon_obs_dist = 1e9;
        for (int i = 0; i < (int)ref_speed.size() && i < (int)obs_points.size(); i++) {
            for (const auto& obs : obs_points[i]) {
                double d = (ref_trajectory[i].head<2>() - obs.head<2>()).norm();
                if (d < min_horizon_obs_dist) min_horizon_obs_dist = d;
            }
        }

        double horizon_scale = 1.0;
        if (min_horizon_obs_dist < OBS_SLOW_DIST) {
            horizon_scale = OBS_MIN_SCALE
                          + (1.0 - OBS_MIN_SCALE) * (min_horizon_obs_dist / OBS_SLOW_DIST);
        }

        for (int i = 0; i < (int)ref_speed.size() && i < (int)obs_points.size(); i++) {
            double step_scale = 1.0;
            if (!obs_points[i].empty()) {
                double min_obs_dist = 1e9;
                for (const auto& obs : obs_points[i]) {
                    double d = (ref_trajectory[i].head<2>() - obs.head<2>()).norm();
                    if (d < min_obs_dist) min_obs_dist = d;
                }
                if (min_obs_dist < OBS_SLOW_DIST) {
                    step_scale = OBS_MIN_SCALE
                              + (1.0 - OBS_MIN_SCALE) * (min_obs_dist / OBS_SLOW_DIST);
                }
            }
            double effective_scale = std::min(step_scale, horizon_scale);
            ref_speed[i] *= effective_scale;
        }
    }

}

void LocalPlanner::rcvGlobalTrajectory(const trajectory_generation::trajectoryPolyConstPtr& polytraj)
{
    ROS_INFO("[Local Planner] rcv global trajectory");
    reference_path.clear();
    reference_velocity.clear();
    duration_time.clear();
    ref_time.clear();
    tracking_low_check_flag.clear();

    speed_direction = 1;
    int piece_num = polytraj->duration.size();
    if (piece_num == 0) {
        ROS_ERROR("[LocalPlanner] received trajectory with 0 pieces — ignoring");
        return;
    }
    m_polyMatrix_x.resize(piece_num, 4);
    m_polyMatrix_y.resize(piece_num, 4);
    // 设置每段轨迹的参数

    bool coef_has_nan = false;
    for(int i = 0; i<piece_num; i++)
    {
        duration_time.push_back(polytraj->duration[i]);
        m_polyMatrix_x(i, 0) = polytraj->coef_x[4 * i + 0];
        m_polyMatrix_x(i, 1) = polytraj->coef_x[4 * i + 1];
        m_polyMatrix_x(i, 2) = polytraj->coef_x[4 * i + 2];
        m_polyMatrix_x(i, 3) = polytraj->coef_x[4 * i + 3];
        m_polyMatrix_y(i, 0) = polytraj->coef_y[4 * i + 0];
        m_polyMatrix_y(i, 1) = polytraj->coef_y[4 * i + 1];
        m_polyMatrix_y(i, 2) = polytraj->coef_y[4 * i + 2];
        m_polyMatrix_y(i, 3) = polytraj->coef_y[4 * i + 3];

        // ── NaN diagnostic: check polynomial coefficients on receipt ──
        if (!std::isfinite(polytraj->duration[i]) || polytraj->duration[i] <= 0.0) {
            ROS_ERROR("[LocalPlanner] BAD duration[%d] = %f", i, polytraj->duration[i]);
            coef_has_nan = true;
        }
        for (int c = 0; c < 4; c++) {
            if (!std::isfinite(polytraj->coef_x[4*i+c])) {
                ROS_ERROR("[LocalPlanner] NaN in coef_x[%d][%d] = %f", i, c, polytraj->coef_x[4*i+c]);
                coef_has_nan = true;
            }
            if (!std::isfinite(polytraj->coef_y[4*i+c])) {
                ROS_ERROR("[LocalPlanner] NaN in coef_y[%d][%d] = %f", i, c, polytraj->coef_y[4*i+c]);
                coef_has_nan = true;
            }
        }
    }
    if (coef_has_nan) {
        ROS_ERROR("[LocalPlanner] Polynomial coefficients contain NaN/Inf — discarding trajectory");
        reference_path.clear();
        reference_velocity.clear();
        duration_time.clear();
        ref_time.clear();
        return;
    }
    ROS_INFO("[LocalPlanner] Polynomial coefficients OK (pieces=%d, coef_x size=%zu, coef_y size=%zu)",
             piece_num, polytraj->coef_x.size(), polytraj->coef_y.size());

    getRefTrajectory();  /// 根据轨迹的参数得到采样轨迹和采样速度
    getRefVel();

    // ── NaN diagnostic: check evaluated reference path & velocity ──
    bool eval_has_nan = false;
    for (size_t i = 0; i < reference_path.size(); i++) {
        if (!std::isfinite(reference_path[i].x()) || !std::isfinite(reference_path[i].y())) {
            ROS_ERROR("[LocalPlanner] NaN in evaluated reference_path[%zu] = (%f, %f)",
                      i, reference_path[i].x(), reference_path[i].y());
            eval_has_nan = true;
            break;  // don't spam
        }
    }
    for (size_t i = 0; i < reference_velocity.size(); i++) {
        if (!std::isfinite(reference_velocity[i].x()) || !std::isfinite(reference_velocity[i].y())) {
            ROS_ERROR("[LocalPlanner] NaN in evaluated reference_velocity[%zu] = (%f, %f)",
                      i, reference_velocity[i].x(), reference_velocity[i].y());
            eval_has_nan = true;
            break;
        }
    }
    if (eval_has_nan) {
        ROS_ERROR("[LocalPlanner] Evaluated trajectory has NaN — discarding");
        reference_path.clear();
        reference_velocity.clear();
        duration_time.clear();
        ref_time.clear();
        return;
    }
    ROS_INFO("[LocalPlanner] Evaluated trajectory OK (path pts=%zu, vel pts=%zu)",
             reference_path.size(), reference_velocity.size());

    start_tracking_time = ros::Time::now();
    last_time_reset_ = ros::Time(0);  // Fix 34c: allow immediate time-reset on new trajectory
    m_get_global_trajectory = true;

    // Fix 34d: Force cold-start on new trajectory. The solver's warm-start
    // from the previous trajectory is completely wrong for the new one and
    // causes the first few frames to oscillate violently.
    if (mpcSolverPtr_) {
        mpcSolverPtr_->reset();
    }
    motion_mode = polytraj->motion_mode;
//    motion_mode = 8;
}


void LocalPlanner::init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &_global_map)
{
    nh.param("tracking_node/local_v_max", m_vmax, 6.0);
    nh.param("tracking_node/local_a_max", m_amax, 6.0);
    nh.param("tracking_node/local_w_max", m_wmax, 8.0);
    nh.param("tracking_node/local_j_max", m_jmax, 8.0);
    nh.param("tracking_node/local_vxtl_max", m_vxtl_max, 1.8);
    nh.param("tracking_node/local_axtl_max", m_axtl_max, 2.0);
    nh.param("tracking_node/local_wxtl_max", m_wxtl_max, 4.0);
    nh.param("tracking_node/local_jxtl_max", m_jxtl_max, 3.0);
    nh.param("tracking_node/rho_", rho_, 1.0);
    nh.param("tracking_node/rhoN_", rhoN_, 2.0);
    nh.param("tracking_node/planning_horizon", planning_horizon, 20);
    nh.param("tracking_node/dt", dt, 0.1);
    nh.param("tracking_node/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("tracking_node/taskFile", taskFile, std::string("task.info"));

    global_map = _global_map;

    cv::Mat occ_map3c;
    occ_map3c = cv::imread(occ_file_path);
    std::vector <cv::Mat> channels;
    cv::split(occ_map3c, channels);
    occ_map = channels.at(0);
    occ_map = swellOccMap(occ_map);

    ROS_WARN("m_vmax: %f", m_vmax);

    predictState.resize(planning_horizon);
    predictInput.resize(planning_horizon);
    ref_trajectory.resize(planning_horizon);
    for (int i = 0; i < planning_horizon; ++i) {
        predictInput[i].setZero();
    }

    reference_trajectory_sub = nh.subscribe("/trajectory_generation/global_trajectory", 1, &LocalPlanner::rcvGlobalTrajectory, this);
    redecision_pub = nh.advertise<std_msgs::Bool>("/redecide_flag", 1);
    mpcInterface_.init(taskFile);
//    mpcInterface_.obsConstraintPtr_.reset(new ObsConstraintSet(reference_time, obs_points));
    mpcInterface_.obsConstraintPtr_.reset(new ObsConstraintSet(reference_time, obs_points_t));
//    mpcSolverPtr_.reset(new ocs2::SqpMpc(mpcInterface_.mpcSettings(), mpcInterface_.sqpSettings(), mpcInterface_.getOptimalControlProblem(), mpcInterface_.getInitializer()));
    bufferPrimalSolutionPtr_.reset(new ocs2::PrimalSolution());
}

cv::Mat LocalPlanner::swellOccMap(cv::Mat occ_map)
{
    cv::Mat occ = cv::Mat::zeros(occ_map.rows, occ_map.cols, CV_8UC1);
    // BUG FIX: was hardcoded (0.3 / 0.1 = 3 cells) regardless of actual
    // map resolution.  With resolution=0.05 this gave only 0.15m inflation
    // instead of the intended 0.3m.  Use the global_map's parameters.
    double res = (global_map) ? global_map->getResolution() : 0.05;
    double radius = (global_map) ? global_map->getRobotRadius() : 0.3;
    int swell_num = (int)(radius / res);

    for (int i = 0; i < occ_map.rows; i++) {
        for (int j = 0; j < occ_map.cols; j++) {
            if (occ_map.at<uchar>(i, j) > 10){   // 膨胀直接画圆
                cv::circle(occ, cv::Point(j, i), swell_num, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    ROS_WARN("map swell done");
    return occ;
}

int LocalPlanner::solveNMPC(Eigen::Vector4d state)
{
    ocs2::scalar_array_t desiredTimeTrajectory(planning_horizon);
    ocs2::vector_array_t desiredStateTrajectory(planning_horizon);
    ocs2::vector_array_t desiredInputTrajectory(planning_horizon);
    for(size_t i = 0; i < planning_horizon; i++)  // 设置参考轨迹
    {

        Eigen::Vector4d reference_state = {ref_trajectory[i](0) + 0.0, ref_trajectory[i](1), ref_speed[i], ref_phi[i]};
        desiredStateTrajectory[i] = reference_state;
        desiredTimeTrajectory[i] = reference_time[i];
        desiredInputTrajectory[i] = Eigen::Vector2d::Zero(2);
    }

    // ── Fix 42b: Reference repulsion DISABLED ─────────────────────────
    // Fix 41b's tangent-perpendicular repulsion (and Fix 40a before it)
    // proved counterproductive in corridors:
    //   - In a corridor with walls on both sides, repulsion pushed the
    //     reference away from one wall INTO the barrier zone of the
    //     opposite wall.  The solver then faced conflicting requirements
    //     (track repulsed reference on one side vs avoid barrier on the
    //     other) and produced a compromise that satisfied neither:
    //     min_obs=0.213m, maxDev=2.130m.
    //   - The double speed reduction (step_scale × horizon_scale) pushed
    //     ref_speed to near-zero, so the reference barely moved forward
    //     while the robot at v=0.3 overshot → huge position error.
    //
    // With repulsion DISABLED, the reference is the original polynomial
    // path from the global planner (corridor center).  The barrier alone
    // handles avoidance:
    //   - In a corridor: barrier from both walls pushes toward center,
    //     aligned with tracking cost → solver naturally centers the path.
    //   - For isolated obstacle: barrier creates "go around" gradient,
    //     and the solver picks the lower-cost side.
    // The barrier (μ=20, δ=1.0) is now strong enough relative to Q=150
    // because the obstacle count is much lower (8 sectors ~100 total).
    ocs2::TargetTrajectories targetTrajectories(desiredTimeTrajectory, desiredStateTrajectory, desiredInputTrajectory);

    // ── NaN guard: catch corrupt reference before feeding it to the solver ──
    for (size_t i = 0; i < planning_horizon; i++) {
        if (!desiredStateTrajectory[i].allFinite()) {
            ROS_ERROR("[LocalPlanner] NaN/Inf in reference state at step %zu: [%f %f %f %f]",
                      i, desiredStateTrajectory[i](0), desiredStateTrajectory[i](1),
                      desiredStateTrajectory[i](2), desiredStateTrajectory[i](3));
            return 0;
        }
    }

    mpcInterface_.getReferenceManagerPtr()->setTargetTrajectories(targetTrajectories);
    mpcSolverPtr_->getSolverPtr()->setReferenceManager(mpcInterface_.getReferenceManagerPtr());
    // Fix 35b: Do NOT negate observation speed by speed_direction.
    // The solver must see the robot's true physical speed (always >= 0).
    // Negating it made the solver think the robot was moving backward when
    // speed_direction == -1, producing overcorrection and violent oscillation.
    // speed_direction now only affects the *reference* (ref_speed, ref_phi),
    // not the observation.
    observation.state = state;

    if (!observation.state.allFinite()) {
        ROS_ERROR("[LocalPlanner] NaN/Inf in observation state: [%f %f %f %f]",
                  state(0), state(1), state(2), state(3));
        return 0;
    }

    bool controllerIsUpdated;
    try {
        controllerIsUpdated = mpcSolverPtr_->run(observation.time, observation.state);
    } catch (const std::exception& e) {
        ROS_ERROR("[LocalPlanner] SQP solver failed: %s - skipping this cycle", e.what());
        // Clear corrupted primal solution so the next cycle cold-starts
        // instead of warm-starting from the failed state (which would
        // cause every subsequent solve to fail too).
        mpcSolverPtr_->reset();
        return 0;
    }
    if (!controllerIsUpdated) {
        return 0;
    }
    ocs2::scalar_t final_time = observation.time + mpcSolverPtr_->settings().solutionTimeWindow_;
    mpcSolverPtr_->getSolverPtr()->getPrimalSolution(final_time, bufferPrimalSolutionPtr_.get());

    // ── Fix 40g: Detailed diagnostic logging ───────────────────────
    // Log obstacle proximity for predicted trajectory so we can verify
    // the collision avoidance is working.
    {
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {  // every ~0.5s at 40Hz
            double min_pred_obs_dist = 1e9;
            int min_pred_step = -1;
            const auto& predTraj = bufferPrimalSolutionPtr_->stateTrajectory_;
            for (size_t i = 0; i < std::min(predTraj.size(), obs_points.size()); i++) {
                for (const auto& obs : obs_points[i]) {
                    double d = (predTraj[i].head<2>() - obs.head<2>()).norm();
                    if (d < min_pred_obs_dist) {
                        min_pred_obs_dist = d;
                        min_pred_step = (int)i;
                    }
                }
            }
            // Also compute max deviation of predicted path from reference
            double max_ref_pred_dev = 0.0;
            for (size_t ii = 0; ii < std::min(predTraj.size(), (size_t)planning_horizon); ii++) {
                if (ii < ref_trajectory.size()) {
                    double dev = (predTraj[ii].head<2>() - ref_trajectory[ii].head<2>()).norm();
                    if (dev > max_ref_pred_dev) max_ref_pred_dev = dev;
                }
            }
            // Also compute near-term max deviation (steps 0-5) which
            // is what actually matters for control quality.
            double max_near_dev = 0.0;
            for (size_t ii = 0; ii < std::min(predTraj.size(), (size_t)5); ii++) {
                if (ii < ref_trajectory.size()) {
                    double dev = (predTraj[ii].head<2>() - ref_trajectory[ii].head<2>()).norm();
                    if (dev > max_near_dev) max_near_dev = dev;
                }
            }
            // Fix 43d: Added refGap metric (distance from robot to ref[0])
            double ref_gap = ref_trajectory.empty() ? 0.0
                : std::hypot(state(0) - ref_trajectory[0](0), state(1) - ref_trajectory[0](1));
            ROS_INFO("[Fix43] obs=%d min_obs=%.3fm@s%d maxDev=%.3f/%.3fm refGap=%.3fm spd0=%.2f v=%.2f pos=[%.2f,%.2f]",
                     obs_num, min_pred_obs_dist, min_pred_step,
                     max_near_dev, max_ref_pred_dev, ref_gap,
                     ref_speed.empty() ? 0.0 : ref_speed[0],
                     state(2), state(0), state(1));
        }
    }

    return 1;
}


bool LocalPlanner::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / 0.1;
    for(int i = 0; i<n+1; i++)
    {
        ray_ptx = p2_x + i * 0.1 * x_offset / distance;
        ray_pty = p2_y + i * 0.1 * y_offset / distance;
        ray_ptz = 0.0;

        int idx, idy, idz;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, idx, idy, idz);
        if (global_map->isOccupied(idx, idy, idz)){
            return false;
        }
    }
    return true;
}


void LocalPlanner::getNMPCPredictXU(Eigen::Vector4d &predict_state)
{
    predict_state(0) = bufferPrimalSolutionPtr_->stateTrajectory_[1](2);
    predict_state(1) = bufferPrimalSolutionPtr_->stateTrajectory_[1](3);
    predict_state(2) = bufferPrimalSolutionPtr_->inputTrajectory_[0](0);
    predict_state(3) = bufferPrimalSolutionPtr_->inputTrajectory_[0](1);
}


void LocalPlanner::getNMPCPredictionDeque(std::vector<Eigen::Vector4d> &state_deque, std::vector<Eigen::Vector2d> &input_deque)
{
    const size_t N = bufferPrimalSolutionPtr_->timeTrajectory_.size();
    for(int i = 0;i<N;i++)
    {
        state_deque.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i]);
//        std::cout<<"velocity: "<<bufferPrimalSolutionPtr_->stateTrajectory_[i][2]<<std::endl;
        input_deque.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i]);
        predictState.push_back(bufferPrimalSolutionPtr_->stateTrajectory_[i]);
        predictInput.push_back(bufferPrimalSolutionPtr_->inputTrajectory_[i]);
    }
}

bool LocalPlanner::checkfeasible()
{
    // Fix 36a/b: Only check dynamic obstacles (isLocalOccupied) — static walls
    // are already handled by the MPC collision barrier.  Checking isOccupied()
    // (static + dynamic) caused 92% false-positive rate because the predicted
    // path naturally deviates slightly toward inflated static walls, triggering
    // perpetual replanning that killed velocity.
    // Fix 42e: Reduced horizon from 50% to 25% (steps 1-5, covering 0.5s).
    // Fix 48: Only flag dynamic-only obstacles.
    //
    // Fix 55: Expand static-map skip to ±2 cell neighborhood.
    // The exact-cell check (data[flat]==1) misses static walls when lidar
    // quantization places the hit 1-2 cells away from the static map entry.
    // This caused perpetual "MPC is not safe" → replan every ~5s → robot
    // moves-pauses-changes-direction.  Checking a ±2 neighborhood eliminates
    // the grid-quantization false positives.
    // Fix 55/55b/60: Static wall neighborhood check.
    // STATIC_CHECK_RADIUS=8 was ±0.4m.  Localization drift can exceed 0.4m
    // near map edges or after fast motion → lidar walls displaced by >0.4m
    // from static map → false dynamic obstacle detection → perpetual replan.
    // Fix 60: Increase to 12 (±0.6m) for better localization drift tolerance.
    static const int STATIC_CHECK_RADIUS = 12;  // (Fix 60) was 8
    for(int i = 1; i < (int)(predictState.size() * 0.25); i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->isLocalOccupied(idx, idy, idz)){
            // Fix 61c: Use precomputed dilated static map instead of O(625) inner loop
            if (idx >= 0 && idx < global_map->GLX_SIZE &&
                idy >= 0 && idy < global_map->GLY_SIZE &&
                global_map->near_static_data[idx * global_map->GLY_SIZE + idy]) {
                continue;  // static wall seen by lidar — not a new obstacle
            }
            return true;
        }
    }
    return false;
}

bool LocalPlanner::checkBridge() {

    for(int i = 0; i<(predictState.size() - 1); i++) {
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height) {  // 在桥洞里不准转云台
            return true;
        }
    }
    return false;
}

bool LocalPlanner::checkXtl(){
    // 检查地形是否可以小陀螺
    double predictPathLength = 0.0;
    if(ref_trajectory.size() < 1){
        return true;
    }

    for(int i = 0; i < (predictState.size() - 1)/2; i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height && global_map->GridNodeMap[idx][idy]->height < 0.4){  // 在桥洞里不准陀螺
            std::cout<<"in bridge"<<std::endl;
            return false;
        }


        Eigen::Vector3d sample_point, sample_point_next;
        sample_point.x() = predictState[i](0);
        sample_point.y() = predictState[i](1);
        sample_point_next.x() = predictState[i+1](0);
        sample_point_next.y() = predictState[i+1](1);

        Eigen::Vector3i sample_index = global_map->coord2gridIndex(sample_point);
        Eigen::Vector3i sample_index_next = global_map->coord2gridIndex(sample_point_next);
        double height_delta = global_map->GridNodeMap[sample_index.x()][sample_index.y()]->height -
                              global_map->GridNodeMap[sample_index_next.x()][sample_index_next.y()]->height;
        if(abs(height_delta) < 0.03){  // TODO 判断有点小问题
            continue;
        }

        predictPathLength = std::sqrt(std::pow(predictState[i+1](0) - predictState[i](0), 2) + std::pow(ref_trajectory[i+1](1) - ref_trajectory[i](1), 2));
        if(predictPathLength < 0.05){
            continue;
        }

        double slope = abs(height_delta) / predictPathLength;

        if(abs(slope) > 0.3){
            std::cout<<"in slope one"<<std::endl;
            return false;
        }
//        predictPathLength += std::sqrt(std::pow(predictState[i+1](0) - predictState[i](0), 2) + std::pow(predictState[i+1](1) - predictState[i](1), 2));

    }

    for(int i = 0; i < (predictState.size())/2; i++){
        int idx, idy, idz;
        double pt_z = 0.0;
        global_map->coord2gridIndex(predictState[i](0), predictState[i](1), pt_z, idx, idy, idz);
        if (global_map->GridNodeMap[idx][idy]->exist_second_height && global_map->GridNodeMap[idx][idy]->height < 0.4){  // 在桥洞里不准陀螺
            std::cout<<"in bridge"<<std::endl;
            return false;
        }


        Eigen::Vector3d sample_point, sample_point_next;
        sample_point.x() = predictState[i](0);
        sample_point.y() = predictState[i](1);
        sample_point_next.x() = predictState[i+4](0);
        sample_point_next.y() = predictState[i+4](1);

        Eigen::Vector3i sample_index = global_map->coord2gridIndex(sample_point);
        Eigen::Vector3i sample_index_next = global_map->coord2gridIndex(sample_point_next);
        double height_delta = global_map->GridNodeMap[sample_index.x()][sample_index.y()]->height -
                              global_map->GridNodeMap[sample_index_next.x()][sample_index_next.y()]->height;
        if(abs(height_delta) > 0.1){  // TODO 判断有点小问题
            return false;
        }
    }

    return true;
}

void LocalPlanner::getRefTrajectory()
{
    double time = accumulate(duration_time.begin(), duration_time.end(), 0.0);
    for(int i = 0; i<=(int)(time/dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i*dt, index, total_time);
//        std::cout<<" total_time: "<<total_time<<" i*dt: "<<i*dt<<" index: "<<index<<" time: "<<time<<std::endl;
//        std::cout<<"(i - (int)(total_time/dt)) * dt: "<< (i - (int)(total_time/dt)) * dt<<std::endl;

        Eigen::Vector3d ref_point;  //
        ref_point(0) = m_polyMatrix_x(index, 0) * pow(i*dt - total_time, 3) + m_polyMatrix_x(index, 1) * pow(
                i*dt - total_time, 2) + m_polyMatrix_x(index, 2) * (i*dt - total_time) + m_polyMatrix_x(
                index, 3);
        ref_point(1) = m_polyMatrix_y(index, 0) * pow(i*dt - total_time, 3) + m_polyMatrix_y(index, 1) * pow(
                i*dt - total_time, 2) + m_polyMatrix_y(index, 2) * (i*dt - total_time) + m_polyMatrix_y(
                index, 3);
        ref_point(2) = 0.0;
        reference_path.push_back(ref_point);
//        ROS_ERROR("reference_path[i].x() %f:  reference_path[i].y() %f: ", reference_path[i].x(), reference_path[i].y());
    }

    Eigen::Vector3d ref_point;  // (i - (int)(total_time/dt)) * dt：区段时间
    ref_point(0) = m_polyMatrix_x(duration_time.size() - 1, 0) * pow(duration_time.back(), 3) + m_polyMatrix_x(duration_time.size() - 1, 1) * pow(
            duration_time.back(), 2) + m_polyMatrix_x(duration_time.size() - 1, 2) * duration_time.back() + m_polyMatrix_x(
            duration_time.size() - 1, 3);
    ref_point(1) = m_polyMatrix_y(duration_time.size() - 1, 0) * pow(duration_time.back(), 3) + m_polyMatrix_y(duration_time.size() - 1, 1) * pow(
            duration_time.back(), 2) + m_polyMatrix_y(duration_time.size() - 1, 2) * duration_time.back() + m_polyMatrix_y(
            duration_time.size() - 1, 3);
    ref_point(2) = 0.0;
    reference_path.push_back(ref_point);
    target_point = reference_path.back();
}

void LocalPlanner::getRefVel()
{
    double time = accumulate(duration_time.begin(), duration_time.end(), 0.0);  /// 参考轨迹的时间分配
    for (int i = 0; i<=(int)(time/dt); i++)
    {
        int index;
        double total_time;
        getSegmentIndex(i*dt, index, total_time);
        Eigen::Vector3d ref_point;
        ref_point(0) = 3 * m_polyMatrix_x(index, 0) * pow((i*dt - total_time), 2) + 2 * m_polyMatrix_x(index, 1) * pow(
                (i*dt - total_time), 1) + m_polyMatrix_x(index, 2);
        ref_point(1) = 3 * m_polyMatrix_y(index, 0) * pow((i*dt - total_time), 2) + 2 * m_polyMatrix_y(index, 1) * pow(
                (i*dt - total_time), 1) + m_polyMatrix_y(index, 2);
        ref_point(2) = 0.0;
        reference_velocity.push_back(ref_point);
        ref_time.push_back(dt);
//        std::cout<<"vx: "<<ref_point(0)<<" vy: "<<ref_point(1)<<std::endl;
    }

    ref_time.push_back(dt);
}

void LocalPlanner::getSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < duration_time.size(); i++) {
        sum_time += duration_time[i];
        if (sum_time >= time) {
            segment_index = i;   // 找到该时间对应的索引段和前几段的总时间
            total_time = sum_time - duration_time[i];
            break;
        }
    }
}

void LocalPlanner::getTrackingSegmentIndex(double time, int &segment_index, double &total_time)
{
    double sum_time = 0.0;
    for (int i = 0; i < ref_time.size(); i++) {
        sum_time += ref_time[i];
        if (sum_time >= time) {
            segment_index = i;   // 找到该时间对应的索引段和前几段的总时间
            total_time = sum_time - ref_time[i];
            break;
        }
        else
        {
            segment_index = ref_time.size();
            total_time = sum_time;
        }
    }
}

void LocalPlanner::getCircleEvadeTraj(Eigen::Vector3d cur_position, std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<Eigen::Vector3d> &ref_velocity, std::vector<double> &reference_time){
    target_point = reference_path.back();  // 终点
    double theta = atan2((cur_position - target_point).y(), (cur_position - target_point).x());
    for(int i = 0; i < 20; i++){
        Eigen::Vector3d velocity_temp;
        Eigen::Vector3d position_temp;
        double delta_theta = (double) 0.1625; // (v*0.1/R)

        theta = theta + delta_theta;
        position_temp.x() = target_point.x() + 0.8 * cos(theta);
        position_temp.y() = target_point.y() + 0.8 * sin(theta);
        position_temp.z() = 0.0;

        velocity_temp.x() = 1.3 * sin(-theta);  // TODO 躲避障碍有点问题，终点判断可能有问题，以及巡航模式下的重规划
        velocity_temp.y() = 1.3 * cos(theta);
        velocity_temp.z() = 0.0;

        ref_trajectory.push_back(position_temp);
        ref_velocity.push_back(velocity_temp);

        reference_time.push_back(i * 0.1);

    }
}