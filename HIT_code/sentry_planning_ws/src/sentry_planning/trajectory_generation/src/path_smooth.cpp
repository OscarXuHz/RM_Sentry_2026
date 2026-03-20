#include "../include/path_smooth.h"
#include <iostream>


Smoother::~Smoother()
{}

void Smoother::setGlobalMap(std::shared_ptr<GlobalMap> &_global_map)
{
    global_map = _global_map;
}

void Smoother::init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed)
{
    // Reset per-path obstacle cache so getObsEdge() resamples obstacles at the
    // new control-point positions. Without this reset, replanning calls reuse
    // obstacle data from the previous (now stale) path, allowing the smoothed
    // path to pass through walls.
    init_obs = false;
    allobs.clear();
    mid_distance.clear();

    pathSample(global_path, start_vel);
    if(path.size()<2){
        ROS_ERROR("[Smooth Init] path.size()<2!!  No path");
        return;
    }
    desire_veloity = desire_speed;
    headP = path[0];
    Eigen::Vector2d start_vels;
    start_vels.x() = start_vel.x();
    start_vels.y() = start_vel.y();

    getGuidePath(start_vels);
    if(!init_vel){  // 速度方向反向过大直接倒车不拐弯
        start_vels.x() = 0.0;
        start_vels.y() = 0.0;
    }

    tailP = path[path.size() - 1];
    pieceN = path.size() - 1;

    // Store head velocity for MINCO
    headV_stored = start_vels;
    // Approximate head acceleration as zero (consistent with boundary conditions)
    headA_stored = Eigen::Vector2d::Zero();

    // MINCO: set boundary conditions with position, velocity, acceleration
    // Tail velocity and acceleration are zero (stop at goal)
    mincoTraj.setConditions(headP, headV_stored, headA_stored,
                            tailP, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                            pieceN);
}

bool Smoother::getObsPosition(double start_x, double start_y, double start_z,
                    double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt)
{
    double path_len = std::sqrt((edge_x - start_x) * (edge_x - start_x) + (edge_y - start_y) * (edge_y - start_y));
    int n = static_cast<int>(path_len * global_map->m_inv_resolution * 2) + 1;
    double delta_x = (edge_x - start_x) / n;
    double delta_y = (edge_y - start_y) / n;
    int x_idx, y_idx, z_idx;

    for(int i = 0; i < n; i++){
        double temp_x = start_x + delta_x * i;
        double temp_y = start_y + delta_y * i;
        double temp_z = start_z;
        /* 用空间点坐标找到其所对应的栅格 */
        global_map->coord2gridIndex(temp_x, temp_y, temp_z, x_idx, y_idx, z_idx);
        if (!isFree(x_idx, y_idx, z_idx)) {
            obs_pt = {temp_x, temp_y, temp_z};
            return true;
        }
    }
    return false;

}

double Smoother::costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
    auto instance = reinterpret_cast<Smoother *>(ptr);
    const int points_num = instance->pieceN - 1;
    const int N = instance->pieceN;
    double cost = 0.0;

    // ── Parse decision variables ──
    // First 2*(N-1) entries: inner waypoint positions (x, y)
    // Last N entries: unconstrained tau_i mapped to T_i via forwardT()
    Eigen::Matrix2Xd inPs;
    inPs.resize(2, points_num);
    inPs.row(0) = x.head(points_num);
    inPs.row(1) = x.segment(points_num, points_num);

    // Map unconstrained tau -> positive durations T via diffeomorphism
    Eigen::VectorXd tau = x.tail(N);
    Eigen::VectorXd inTimes;
    forwardT(tau, inTimes);

    // ── Build MINCO trajectory ──
    instance->mincoTraj.setParameters(inPs, inTimes);

    // ── Initialize/refresh obstacle cache ──
    // (Fix 30) Refresh every 50 iterations so the optimizer sees obstacles
    // near the waypoints' CURRENT positions, not stale initial positions.
    // The refresh only rebuilds the cache; gradient computation happens below.
    instance->cost_call_count++;
    if(!instance->init_obs || (instance->cost_call_count % 50 == 0)) {
        instance->allobs.clear();
        instance->mid_distance.clear();
        instance->init_obs = false;
        for (int i = 0; i < points_num; ++i) {
            double nearest_cost_dummy = 0.0;
            Eigen::Vector2d x_cur = inPs.col(i);
            // This call with init_obs=false populates allobs[i] and mid_distance[i]
            instance->obstacleTerm(i, x_cur, nearest_cost_dummy);
        }
        instance->init_obs = true;
        // Cache is now fresh — fall through to gradient computation below
    }

    // ── Cost weights ──
    // (Fix 31) Rebalanced to reduce zigzag near turns and endpoints.
    // Previous wFidelity=500 anchored EVERY 0.3m waypoint to its A* reference,
    // preventing the smoother from rounding corners. Reduced to 100 so the
    // smoother can actually smooth turns. The 0.5m drift clamp remains as the
    // hard safety limit, not fidelity. wSmooth raised to 5e-3 for stronger
    // jerk reduction near turns.
    const double wSmooth = 2e-2;      // jerk energy weight (strongly smooths corners)
    const double wFidelity = 20.0;    // elastic band — weak bias, allows corner rounding
    const double wVel = 1e3;          // average velocity soft constraint
    const double wMinTime = 1e3;      // minimum-time penalty
    const double energyGradClip = 2000.0;  // per-waypoint energy gradient norm cap

    // ── Jerk energy cost and gradients ──
    double energy = instance->mincoTraj.getEnergy();
    // Guard against NaN/Inf energy (can happen with degenerate T)
    if (!std::isfinite(energy)) {
        g.setZero();
        return 1e18;
    }
    cost += wSmooth * energy;

    Eigen::Matrix2Xd energy_grad;
    instance->mincoTraj.getGradWaypoints(energy_grad);
    energy_grad *= wSmooth;

    // Clip per-waypoint energy gradient to prevent domination from
    // short-T segments (where 1/T^n terms explode)
    for (int i = 0; i < points_num; i++) {
        double eg_norm = energy_grad.col(i).norm();
        if (eg_norm > energyGradClip) {
            energy_grad.col(i) *= energyGradClip / eg_norm;
        }
    }

    Eigen::VectorXd energyT_grad;
    instance->mincoTraj.getGradTimes(energyT_grad);
    energyT_grad *= wSmooth;
    // Clip time gradient too
    for (int i = 0; i < N; i++) {
        energyT_grad(i) = std::max(-energyGradClip, std::min(energyGradClip, energyT_grad(i)));
    }

    // Chain rule: grad_tau = grad_T * dT/dtau via backwardGradT
    Eigen::VectorXd energyTau_grad;
    backwardGradT(tau, energyT_grad, energyTau_grad);

    // ── Elastic band / path fidelity cost ──
    // Anchors waypoints to their original A* reference positions
    Eigen::Matrix2Xd fidelity_grad;
    fidelity_grad.resize(2, points_num);
    fidelity_grad.setZero();
    for (int i = 0; i < points_num; i++) {
        Eigen::Vector2d diff = inPs.col(i) - instance->refWaypoints.col(i);
        cost += wFidelity * diff.squaredNorm();
        fidelity_grad.col(i) = 2.0 * wFidelity * diff;
    }

    // ── Average velocity penalty (with proper gradient) ──
    // Penalizes segments whose average velocity (seg_length / T_i) exceeds limit
    Eigen::Matrix2Xd vel_spatial_grad;
    vel_spatial_grad.resize(2, points_num);
    vel_spatial_grad.setZero();
    Eigen::VectorXd vel_time_grad;
    vel_time_grad.resize(N);
    vel_time_grad.setZero();

    double v_limit = instance->desire_veloity * 1.5;
    for (int i = 0; i < N; i++) {
        Eigen::Vector2d p_start = (i == 0) ? instance->headP : inPs.col(i - 1);
        Eigen::Vector2d p_end   = (i == N - 1) ? instance->tailP : inPs.col(i);
        Eigen::Vector2d seg = p_end - p_start;
        double seg_len = seg.norm();
        if (seg_len < 1e-8) continue;

        double avg_vel = seg_len / inTimes(i);
        if (avg_vel > v_limit) {
            double excess = avg_vel - v_limit;
            cost += wVel * excess * excess;

            // Gradient w.r.t. segment endpoints
            Eigen::Vector2d dir = seg / seg_len;
            double d_avgvel_d_len = 1.0 / inTimes(i);
            Eigen::Vector2d grad_pend   =  2.0 * wVel * excess * d_avgvel_d_len * dir;
            Eigen::Vector2d grad_pstart = -2.0 * wVel * excess * d_avgvel_d_len * dir;

            // p_end is inPs.col(i) unless i == N-1 (tailP, fixed)
            if (i < N - 1)  vel_spatial_grad.col(i) += grad_pend;
            // p_start is inPs.col(i-1) unless i == 0 (headP, fixed)
            if (i > 0)      vel_spatial_grad.col(i - 1) += grad_pstart;

            // Gradient w.r.t. time: d(avg_vel)/dT = -seg_len / T^2
            vel_time_grad(i) += 2.0 * wVel * excess * (-seg_len / (inTimes(i) * inTimes(i)));
        }
    }

    // Chain velocity time gradient through tau mapping
    Eigen::VectorXd velTau_grad;
    backwardGradT(tau, vel_time_grad, velTau_grad);

    // ── Minimum time penalty ──
    // Prevents durations from collapsing (causes coefficient blow-up)
    Eigen::VectorXd mintime_T_grad;
    mintime_T_grad.resize(N);
    mintime_T_grad.setZero();
    const double T_min = 0.03;
    for (int i = 0; i < N; i++) {
        if (inTimes(i) < T_min) {
            double deficit = T_min - inTimes(i);
            cost += wMinTime * deficit * deficit;
            mintime_T_grad(i) = -2.0 * wMinTime * deficit;
        }
    }
    Eigen::VectorXd mintimeTau_grad;
    backwardGradT(tau, mintime_T_grad, mintimeTau_grad);

    // ── Obstacle cost (unchanged) ──
    Eigen::Matrix2Xd potential_grad;
    potential_grad.resize(2, points_num);
    potential_grad.setZero();
    for(int i = 0; i < points_num; ++i){
        double nearest_cost = 0.0;
        Eigen::Vector2d x_cur = inPs.col(i);
        Eigen::Vector2d obstacleGrad = instance->obstacleTerm(i, x_cur, nearest_cost);
        potential_grad.col(i) = obstacleGrad;
        cost += nearest_cost;
    }

    // ── Assemble total gradient ──
    Eigen::Matrix2Xd total_spatial_grad = energy_grad + potential_grad
                                        + fidelity_grad + vel_spatial_grad;

    g.setZero();
    // Spatial gradients: first 2*(N-1) entries
    g.head(points_num) = total_spatial_grad.row(0).transpose();
    g.segment(points_num, points_num) = total_spatial_grad.row(1).transpose();
    // Time gradients: last N entries (in tau space)
    g.tail(N) = energyTau_grad + velTau_grad + mintimeTau_grad;

    // Final NaN/Inf guard — return huge cost with zero gradient so
    // L-BFGS backs off this step
    if (!std::isfinite(cost) || !g.allFinite()) {
        g.setZero();
        return 1e18;
    }

    return cost;
}

void Smoother::pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel)
{
    std::vector<Eigen::Vector2d> trajectory_point;
    double step = 0.3;
    double last_dis = 0.0;  /// 上一段留下来的路径
    path.clear();
    trajectory_point.clear();
    if(global_path.size()<2){
        ROS_ERROR("[Smooth Sample] global_path.size()<2, No path");
        return;
    }

//    if(start_vel.norm() > 1.0){  // 小trick
//        step = 0.3;
//    }

    for (int i = 0;i<global_path.size() - 1;i++)
    {
        Eigen::Vector2d start(global_path[i].x(), global_path[i].y());
        Eigen::Vector2d end(global_path[i+1].x(), global_path[i+1].y());
        Eigen::Vector2d start2end(global_path[i+1].x() - global_path[i].x(), global_path[i+1].y() - global_path[i].y());
        double path_distance = std::sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
        /// 每一段的距离
        if(trajectory_point.empty()){
            trajectory_point.push_back(start);
        }
        /// 将上一段路径残留的部分放进去
        Eigen::Vector2d start_new;
        if(path_distance>=(step - last_dis)){
            start_new.x() = start.x() + start2end.x() * (step - last_dis)/path_distance;
            start_new.y() = start.y() + start2end.y() * (step - last_dis)/path_distance;
//            trajectory_point.push_back(start_new);
            step = 0.3;  // 无论速度多大我们只调整第一段的长度，push_back后直接回归低采样
        }
        else  /// 这段路径太短了，加上last_dis还没有采样步长长，因此直接加到dis_last中
        {
            last_dis = last_dis + path_distance;  /// TODO 这种处理方式有个问题，容易把角点扔掉，不保证安全性
            continue;
        }

        Eigen::Vector2d new_start2end(end.x() - start_new.x(), end.y() - start_new.y());
        double path_distance_new = std::sqrt(pow(start_new.x() - end.x(), 2) + pow(start_new.y() - end.y(), 2));
        int sample_num = (int)((path_distance_new)/step);
        if(path_distance_new < 0.1){
            last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
            trajectory_point.push_back(start_new);
            continue;
        }

        for (int j = 0;j<=sample_num; j++){
            Eigen::Vector2d sample_point(start_new.x() + new_start2end.x() * step/path_distance_new * j,
                            start_new.y() + new_start2end.y() * step/path_distance_new * j);
            trajectory_point.push_back(sample_point);
        }
        last_dis = (path_distance-(step-last_dis)) - ((float)sample_num) * step;
    }

    if(last_dis < step/2.0f && trajectory_point.size()>=2)
    {
        trajectory_point.erase((trajectory_point.end()-1));
    }
    Eigen::Vector2d end(global_path[global_path.size() - 1].x(),global_path[global_path.size() - 1].y());
    trajectory_point.push_back(end);
    path.assign(trajectory_point.begin(), trajectory_point.end());
}

void Smoother::getGuidePath(Eigen::Vector2d start_vel, double radius){
    if(path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }
    Eigen::Vector2d direction = path[1] - path[0];
    Eigen::Vector2d start_point = path[0];
    direction = direction / direction.norm();

    double theta = acos((start_vel.x() * direction.x() + start_vel.y() * direction.y()) / (direction.norm() * start_vel.norm()));

    if(theta < M_PI_2 * 4/3 && start_vel.norm() > 0.2){
        init_vel = true;
    }else{
        init_vel = false;
    }
}

void Smoother::smoothPath()
{
    allobs.clear();
    mid_distance.clear();
    init_obs = false;
    init_vel = false;
    cost_call_count = 0;  // (Fix 30) reset iteration counter for obstacle cache refresh
    // MINCO joint space-time optimization: 2*(N-1) spatial + N time variables
    if(path.size()<3){
        ROS_ERROR("[Smooth Path] path.size()<3, No need to optimize");
        return;
    }

    // Decision variable: [x_1..x_{N-1}, y_1..y_{N-1}, tau_1..tau_N]
    const int spatial_dim = 2 * (pieceN - 1);
    const int total_dim = spatial_dim + pieceN;
    Eigen::VectorXd x(total_dim);

    // Initialize spatial variables from sampled path
    for(int i = 0; i < pieceN - 1; i++){
        x(i) = path[i + 1].x();
        x(i + pieceN - 1) = path[i + 1].y();
    }

    // Store reference waypoints for elastic band cost
    refWaypoints.resize(2, pieceN - 1);
    for(int i = 0; i < pieceN - 1; i++){
        refWaypoints(0, i) = path[i + 1].x();
        refWaypoints(1, i) = path[i + 1].y();
    }

    // Initialize time variables proportional to segment length
    Eigen::VectorXd T_init(pieceN);
    for(int i = 0; i < pieceN; i++){
        double seg_len = (path[i + 1] - path[i]).norm();
        T_init(i) = std::max(seg_len / desire_veloity, 0.05);
    }
    Eigen::VectorXd tau_init;
    backwardT(T_init, tau_init);
    x.tail(pieceN) = tau_init;

    obs_coord.clear();

    double minCost = 0.0;
    lbfgs_params.mem_size = 64;
    lbfgs_params.past = 5;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 2.0e-5;
    lbfgs_params.delta = 2e-5;
    lbfgs_params.max_linesearch = 32;
    lbfgs_params.f_dec_coeff = 1.0e-4;
    lbfgs_params.s_curv_coeff = 0.9;
    lbfgs_params.max_iterations = 200;  // Prevent infinite loop
    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &Smoother::costFunction,
                                    nullptr,
                                    this,
                                    lbfgs_params);

    if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH
        || ret == lbfgs::LBFGSERR_MAXIMUMITERATION)
    {
        if(ret > 0){
            ROS_DEBUG_STREAM("[Smooth Optimize] MINCO Optimization Success: "
                             << lbfgs::lbfgs_stderr(ret));
        }else if (ret == 0){
            ROS_INFO_STREAM("[Smooth Optimize] MINCO Optimization STOP: "
                                     << lbfgs::lbfgs_stderr(ret));
        }
        else if (ret == lbfgs::LBFGSERR_MAXIMUMITERATION){
            ROS_INFO("[Smooth Optimize] MINCO reached max iterations (200)");
        }
        else{
            ROS_INFO_STREAM("[Smooth Optimize] MINCO reaches max evaluations: "
                                    << lbfgs::lbfgs_stderr(ret));
        }

        // Extract optimized spatial variables — clamp drift from reference
        double max_drift = 0.0;
        for(int i = 0; i < pieceN - 1; i++){
            double drift_x = x(i) - refWaypoints(0, i);
            double drift_y = x(i + pieceN - 1) - refWaypoints(1, i);
            double drift = std::sqrt(drift_x * drift_x + drift_y * drift_y);
            max_drift = std::max(max_drift, drift);
            // Clamp: obstacle cache refreshes every 50 iters (Fix 30) so
            // waypoints can safely drift further. 0.8m allows corner rounding.
            const double clamp_radius = 0.8;
            if (drift > clamp_radius) {
                double scale = clamp_radius / drift;
                x(i) = refWaypoints(0, i) + drift_x * scale;
                x(i + pieceN - 1) = refWaypoints(1, i) + drift_y * scale;
            }
            path[i + 1].x() = x(i);
            path[i + 1].y() = x(i + pieceN - 1);
        }
        pathInPs.resize(2, pieceN - 1);
        pathInPs.row(0) = x.head(pieceN - 1);
        pathInPs.row(1) = x.segment(pieceN - 1, pieceN - 1);

        // Extract optimized time durations: tau -> T via forwardT
        Eigen::VectorXd tau_opt = x.tail(pieceN);
        Eigen::VectorXd T_opt;
        forwardT(tau_opt, T_opt);

        // Post-process: enforce velocity feasibility by scaling up durations
        // Rebuild trajectory with optimized values to evaluate velocities
        Eigen::Matrix2Xd finalInPs;
        finalInPs.resize(2, pieceN - 1);
        finalInPs.row(0) = x.head(pieceN - 1);
        finalInPs.row(1) = x.segment(pieceN - 1, pieceN - 1);
        mincoTraj.setParameters(finalInPs, T_opt);

        double v_max_limit = desire_veloity * 1.8;
        double max_velocity = 0.0;
        for(int i = 0; i < pieceN; i++){
            double maxVel = 0.0;
            for(int s = 0; s <= 10; s++){
                double t = T_opt(i) * s / 10.0;
                double vel = mincoTraj.evaluateVel(i, t).norm();
                if(vel > maxVel) maxVel = vel;
            }
            if(maxVel > max_velocity) max_velocity = maxVel;
            if(maxVel > v_max_limit && maxVel > 1e-6){
                // Cap inflation factor to prevent time explosion
                double inflation = std::min(maxVel / v_max_limit, 5.0);
                T_opt(i) *= inflation;
            }
        }

        // Clamp per-segment times to sane range [0.01, 2.0]
        // Prevents total trajectory time from exploding (which crashes RViz)
        const double T_lower = 0.01;
        const double T_upper = 2.0;
        for(int i = 0; i < pieceN; i++){
            if(T_opt(i) < T_lower) T_opt(i) = T_lower;
            if(T_opt(i) > T_upper) T_opt(i) = T_upper;
        }

        // Store optimized durations for downstream (Refenecesmooth)
        m_trapezoidal_time.clear();
        m_trapezoidal_time.resize(pieceN);
        for(int i = 0; i < pieceN; i++){
            m_trapezoidal_time[i] = T_opt(i);
        }
        ROS_INFO("[Smooth Optimize] MINCO: cost=%.1f total_T=%.3f(init %.3f) "
                 "max_drift=%.3f max_vel=%.1f segs=%d",
                 minCost, T_opt.sum(), T_init.sum(),
                 max_drift, max_velocity, pieceN);
    }
    else
    {
        // Keep the original sampled path as a safe fallback instead of
        // discarding it.  The sampled path already avoids obstacles (it comes
        // from the topo-search + A* pruning), so it is valid for tracking.
        ROS_WARN_STREAM("[Smooth Optimize] MINCO Optimization failed (" 
                        << lbfgs::lbfgs_stderr(ret)
                        << "), falling back to unsmoothed path (" 
                        << path.size() << " pts)");

        // Fallback: populate m_trapezoidal_time with segment-length-based durations
        m_trapezoidal_time.clear();
        m_trapezoidal_time.resize(pieceN);
        for(int i = 0; i < pieceN; i++){
            double seg_len = (path[i + 1] - path[i]).norm();
            m_trapezoidal_time[i] = std::max(seg_len / desire_veloity, 0.05);
        }
    }
}

void Smoother::pathResample()
{
    ROS_INFO("[Smooth Resample] resample");
    finalpath.clear();
    if(path.size() < 2){
        ROS_ERROR("[Smooth Resample] path size < 2, No path");
        return;
    }
    if(path.size() <= 4){
        // Too few points to decimate — keep all
        finalpath.assign(path.begin(), path.end());
        ROS_INFO("[Smooth Resample] path too short (%zu pts), keeping all", path.size());
        return;
    }

    const int step = 2;

    // Build decimated index list: always keep first, every step-th, always keep last
    std::vector<int> keep_indices;
    keep_indices.push_back(0);
    for (int i = step; i < (int)path.size() - 1; i += step) {
        keep_indices.push_back(i);
    }
    keep_indices.push_back((int)path.size() - 1);

    // Build decimated waypoint list
    std::vector<Eigen::Vector2d> trajectory_point;
    for (int idx : keep_indices) {
        trajectory_point.push_back(path[idx]);
    }

    // Merge m_trapezoidal_time: each resampled segment's duration is the
    // sum of the original MINCO durations it spans.
    if (!m_trapezoidal_time.empty() &&
        (int)m_trapezoidal_time.size() == (int)path.size() - 1) {
        std::vector<double> merged_times;
        for (int k = 0; k < (int)keep_indices.size() - 1; k++) {
            double merged_t = 0.0;
            for (int j = keep_indices[k]; j < keep_indices[k+1]; j++) {
                merged_t += m_trapezoidal_time[j];
            }
            merged_times.push_back(std::max(merged_t, 0.01));
        }
        m_trapezoidal_time = merged_times;
    }

    finalpath.assign(trajectory_point.begin(), trajectory_point.end());

    // Log max turning angle for diagnostics
    double max_angle_deg = 0.0;
    for (int i = 1; i < (int)finalpath.size() - 1; i++) {
        Eigen::Vector2d v1 = finalpath[i] - finalpath[i-1];
        Eigen::Vector2d v2 = finalpath[i+1] - finalpath[i];
        double dot = v1.dot(v2);
        double cross = v1.x()*v2.y() - v1.y()*v2.x();
        double angle = std::abs(std::atan2(cross, dot)) * 180.0 / M_PI;
        if (angle > max_angle_deg) max_angle_deg = angle;
    }
    ROS_INFO("[Smooth Resample] %zu -> %zu waypoints, %zu time segs, max_turn=%.1f deg",
             path.size(), finalpath.size(), m_trapezoidal_time.size(), max_angle_deg);
}

std::vector<Eigen::Vector2d> Smoother::getPath() /// 优化过后的最终轨迹控制点
{
    return finalpath;
}

std::vector<Eigen::Vector2d> Smoother::getSamplePath(){  /// 优化前的采样点(可视化用)
    return path;
}

Eigen::Vector2d Smoother::obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost)
{
    nearest_cost = 0.0;
    Eigen::Vector2d gradient(0.0, 0.0);
    double R = 0.3;  /// 防碰撞半径
    if(!init_obs)
    {
        obs_coord.clear();
        getObsEdge(xcur);
        allobs.push_back(obs_coord);

        std::vector<double> distance_temp;
        double ddd = 0.0;
        for(int i = 0; i < obs_coord.size(); i++)
        {
            double distance = std::sqrt((xcur(0) - obs_coord[i](0)) * (xcur(0) - obs_coord[i](0)) +
                                        (xcur(1) - obs_coord[i](1)) * (xcur(1) - obs_coord[i](1)));
            if(distance > (R * 1.2)){
                continue;
            }
            distance_temp.push_back(distance);
            ddd += distance;
        }

        if(!distance_temp.empty()){
            mid_distance.push_back(ddd/distance_temp.size());   /// 均值阈值用于处理狭窄路段
        }else{
            mid_distance.push_back(R);
        }

        return gradient;
    }

    int size = allobs[idx].size();
    for(int i = 0; i < allobs[idx].size(); i++)
    {

        double tempR = mid_distance[idx];
        double distance = std::sqrt((xcur(0) - allobs[idx][i](0)) * (xcur(0) - allobs[idx][i](0)) + (xcur(1) - allobs[idx][i](1)) * (xcur(1) - allobs[idx][i](1)));

        if(distance > tempR || distance < 1e-10){
            continue;
        }

        nearest_cost += wObstacle * (tempR - distance) * (tempR - distance)  / (double)size;
        Eigen::Vector2d obsVct(xcur.x() - allobs[idx][i](0), xcur.y() - allobs[idx][i](1));
        gradient += wObstacle * 2 * (obsVct / distance) * (distance - tempR) / (double)size;
    }
    return gradient;
}


void Smoother::getObsEdge(Eigen::Vector2d xcur)
{
    Eigen::Vector3d start_pt;
    start_pt(0) = xcur(0);
    start_pt(1) = xcur(1);
    start_pt(2) = 0.0;
    Eigen::Vector3i start_idx = global_map->coord2gridIndex(start_pt);
    start_pt = global_map->gridIndex2coord(start_idx);

    double start_x = start_pt(0);
    double start_y = start_pt(1);
    double start_z = 0.0;
    double R = 4.0;

    for(int i = -12; i <= 12; i++) /// (Fix 31) was ±8; widened to ±12 (±0.6m at 0.05m/cell)
        /// to match increased drift clamp (0.8m). Covers obstacle search area.
    {
        for (int j = -12; j <=12; j ++)
        {
            int x_idx = std::min(std::max(start_idx.x() + i, 0), global_map->GLX_SIZE - 1);
            int y_idx = std::min(std::max(start_idx.y() + j, 0), global_map->GLY_SIZE - 1);
            int z_idx = start_idx.z();

            Eigen::Vector3i temp_idx = {x_idx, y_idx, z_idx};

            if (!isFree(x_idx, y_idx, z_idx)) {
                Eigen::Vector3d obs_pt = global_map->gridIndex2coord(temp_idx);
                obs_coord.push_back(obs_pt);
            }
        }
    }

    for(int i = 0; i < 100; i++){   /// 搜索最近2.5m范围内的点
        double edge_x = start_x + sin(i * M_PI / 50) * R;
        double edge_y = start_y + cos(i * M_PI / 50) * R;
        double edge_z = start_z;

        Eigen::Vector3d obs_pt;
        if(getObsPosition(start_x, start_y, start_z, edge_x, edge_y, edge_z, obs_pt))
        {
            if((obs_pt - start_pt).norm() > 1.0){
                obs_coord.push_back(obs_pt);
            }
        }
    }
}


bool Smoother::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return ((global_map->data[idx_x * global_map->GLY_SIZE + idx_y] < 1 && global_map->l_data[idx_x * global_map->GLY_SIZE + idx_y] < 1));
}

bool Smoother::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

void Smoother::forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
{
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++)
    {
        T(i) = tau(i) > 0.0
               ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
               : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
}

void Smoother::backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau)
{
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++)
    {
        tau(i) = T(i) > 1.0
                 ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                 : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }

    return;
}

void Smoother::backwardGradT(const Eigen::VectorXd &tau, Eigen::VectorXd &gradT, Eigen::VectorXd &gradTau)
{
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++){
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
    }
    return;
}
