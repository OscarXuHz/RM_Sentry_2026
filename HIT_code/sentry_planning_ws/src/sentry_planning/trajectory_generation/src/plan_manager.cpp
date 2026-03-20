//
// Created by zzt on 23-9-25.
//
#include "../include/plan_manager.h"

void planner_manager::init(ros::NodeHandle &nh)
{
    nh.param("trajectory_generator/reference_v_max", reference_v_max, 2.8);
    nh.param("trajectory_generator/reference_a_max", reference_a_max, 4.2);
    nh.param("trajectory_generator/reference_w_max", reference_w_max, 3.0);
    nh.param("trajectory_generator/reference_desire_speed", reference_desire_speed, 3.0);
    nh.param("trajectory_generator/reference_desire_speedxtl", reference_desire_speedxtl, 2.0);
    nh.param("trajectory_generator/reference_axtl_max", reference_axtl_max, 2.0);
    nh.param("trajectory_generator/reference_wxtl_max", reference_wxtl_max, 2.0);

    nh.param("trajectory_generator/isxtl", isxtl, false);
    nh.param("trajectory_generator/xtl_flag", xtl_flag, false);
    nh.param("trajectory_generator/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("trajectory_generator/bev_file_path", bev_file_path, std::string("bevfinal.png"));
    nh.param("trajectory_generator/distance_map_file_path", distance_map_file_path, std::string("distance.png"));

    nh.param("trajectory_generator/map_resolution", map_resolution, 0.05);
    nh.param("trajectory_generator/map_x_size", map_x_size, 28.0);
    nh.param("trajectory_generator/map_y_size", map_y_size, 15.0);
    nh.param("trajectory_generator/map_z_size", map_z_size, 3.0);
    nh.param("trajectory_generator/search_height_min", search_height_min, 0.1);
    nh.param("trajectory_generator/search_height_max", search_height_max, 1.2);
    nh.param("trajectory_generator/search_radius", search_radius, 5.0);

    nh.param("trajectory_generator/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("trajectory_generator/robot_radius", robot_radius, 0.35);
    nh.param("/bot_sim/robot_radius", robot_radius, robot_radius);
    nh.param("trajectory_generator/obstacle_swell_flag", obstacle_swell_flag, true);

    nh.param("trajectory_generator/map_lower_point_x", map_lower_point(0), 0.0);
    nh.param("trajectory_generator/map_lower_point_y", map_lower_point(1), 0.0);
    nh.param("trajectory_generator/map_lower_point_z", map_lower_point(2), 0.0);

    ROS_INFO("[Manager Init] map/file_path: %s", map_file_path.c_str());
    ROS_INFO("[Manager Init] map/occ_file_path: %s", occ_file_path.c_str());
    ROS_INFO("[Manager Init] map/bev_file_path: %s", bev_file_path.c_str());
    ROS_INFO("[Manager Init] map/distance_map_file_path: %s", distance_map_file_path.c_str());

    m_eng = std::default_random_engine(m_rd());
    m_rand_pos = std::uniform_real_distribution<double>(-1.0, 1.0);

    sentryColor = teamColor::red;

    map_inv_resolution = 1.0 / map_resolution;
    map_upper_point(0) = map_lower_point(0) + map_x_size;
    map_upper_point(1) = map_lower_point(1) + map_y_size;
    map_upper_point(2) = map_lower_point(2) + map_z_size;
    grid_max_id_x = (int)(map_x_size * map_inv_resolution);
    grid_max_id_y = (int)(map_y_size * map_inv_resolution);
    grid_max_id_z = (int)(map_z_size * map_inv_resolution);

    global_map.reset(new GlobalMap);
    global_map->initGridMap(nh, occ_file_path, bev_file_path, distance_map_file_path, map_resolution,
                            map_lower_point, map_upper_point, grid_max_id_x, grid_max_id_y, grid_max_id_z,
                            robot_radius, search_height_min, search_height_max, search_radius); ///////////////

    astar_path_finder.reset(new AstarPathFinder);
    astar_path_finder->initGridMap(nh, map_resolution, map_lower_point, map_upper_point,
                                   grid_max_id_x, grid_max_id_y, grid_max_id_z, robot_radius,
                                   search_height_min, search_height_max, global_map);

    path_smoother.reset(new Smoother);
    path_smoother->setGlobalMap(global_map);

    reference_path.reset(new Refenecesmooth);
    reference_path->init(global_map);

    topo_prm.reset(new TopoSearcher);
    topo_prm->init(nh, global_map);

    global_map->setRadiusDash(robot_radius_dash);  // 动态障碍物膨胀半径设置
}

bool planner_manager::replanFinding(const Eigen::Vector3d start_point, const Eigen::Vector3d target_point,
                                    const Eigen::Vector3d start_vel)
{
    ROS_INFO("[Manager REPLAN]  cur position point (X, Y) = (%f, %f), target point (X, Y) = (%f, %f)", start_point(0), start_point(1), target_point(0), target_point(1));
    if (optimized_path.size() > 0){
        int path_start_id;
        int path_end_id;
        Eigen::Vector3d collision_pos;
        Eigen::Vector3d collision_start_point;
        Eigen::Vector3d collision_target_point;
        Eigen::Vector3d target_temp = optimized_path.back();  /// 这里的优化路径可能不是我的目标终点
        ROS_INFO("[Manager REPLAN] target_temp (X, Y) = (%f, %f)", target_temp(0), target_temp(1));
        /// 检查碰撞并判断距离规划的最后一个点与目标点的距离，如果原路径碰撞或者可通行且规划终点与实际终点不符的都要进行全局重规划
        bool collision = astar_path_finder->checkPathCollision(optimized_path, collision_pos, start_point,
                                                               collision_start_point, collision_target_point,
                                                               path_start_id, path_end_id);
        double target_distance = sqrt(pow(target_point.x() - target_temp.x(), 2) +
                               pow(target_point.y() - target_temp.y(), 2)) + 0.01;

        if(target_distance > 0.5){
            ROS_WARN("[Manager REPLAN] target_distance: %f", target_distance);
        }
        if (collision) {  // 发生碰撞后的处理
            std::vector<Eigen::Vector3d> local_path;  // 先进行局部规划
            ROS_INFO("[Manager REPLAN] start local planning");
            local_path = localPathFinding(collision_start_point, collision_target_point);

            if ((local_path.size() == 0)) {
                ROS_WARN("[Manager REPLAN] local plan fail need global planning replan");
                if(!pathFinding(start_point, target_point, start_vel)){
                    return false;
                }else{
                    return true;
                }
            }
            else{
                local_path.insert(local_path.end(), optimized_path.begin() + path_end_id + 1, optimized_path.end());
                optimized_path = astar_path_finder->smoothTopoPath(local_path);  // 剪枝优化topo路径;
                local_optimize_path = local_path;
                /* 二次规划（路径裁减） */

                ROS_INFO("[Manager REPLAN] optimized path size is %d", optimized_path.size());
                // 路径优化

                double reference_speed = isxtl? reference_desire_speedxtl : reference_desire_speed;

                path_smoother->init(optimized_path, start_vel, reference_speed);
                path_smoother->smoothPath();
                path_smoother->pathResample();
                final_path = path_smoother->getPath();
//                final_path = path_smoother->getSamplePath();
                if(final_path.size() < 2){
                    return false;
                }
                reference_path->setGlobalPath(start_vel, final_path,reference_a_max, reference_speed, isxtl);
                reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
                if(ref_trajectory.size() < 2){
                    return false;
                }
                return true;
            }
        }
        else{
            if(target_distance > 0.5){
                // Target moved — need full replan to new target
                ROS_WARN("[Manager REPLAN] target moved (%.2fm), full global replan", target_distance);
                // Remember old path length for hysteresis comparison
                double old_len = 0;
                for(size_t k = 1; k < optimized_path.size(); k++)
                    old_len += (optimized_path[k] - optimized_path[k-1]).head<2>().norm();

                std::vector<Eigen::Vector3d> old_path = optimized_path;
                if(!pathFinding(start_point, target_point, start_vel)){
                    optimized_path = old_path;  // restore on failure
                    return false;
                }

                // Hysteresis: keep old path if new path is NOT significantly shorter
                double new_len = 0;
                for(size_t k = 1; k < optimized_path.size(); k++)
                    new_len += (optimized_path[k] - optimized_path[k-1]).head<2>().norm();
                if(old_len > 0 && new_len > old_len * 0.8){
                    // New path is not much shorter — check if old path is still collision-free
                    Eigen::Vector3d dummy_pos, dummy_start, dummy_target;
                    int dummy_sid, dummy_eid;
                    bool old_collision = astar_path_finder->checkPathCollision(
                        old_path, dummy_pos, start_point, dummy_start, dummy_target, dummy_sid, dummy_eid);
                    if(!old_collision){
                        ROS_INFO("[Manager REPLAN] keeping old path (old=%.1fm new=%.1fm, no collision)", old_len, new_len);
                        optimized_path = old_path;
                        // (Fix 14) Trim forward to nearest waypoint ahead of robot
                        {
                            int closest_idx = 0;
                            double min_dist = 1e9;
                            for (int i = 0; i < (int)optimized_path.size(); i++) {
                                double d = (optimized_path[i].head<2>() - start_point.head<2>()).norm();
                                if (d < min_dist) {
                                    min_dist = d;
                                    closest_idx = i;
                                }
                            }
                            int trim_idx = std::min(closest_idx + 1, (int)optimized_path.size() - 1);
                            std::vector<Eigen::Vector3d> trimmed;
                            trimmed.push_back(start_point);
                            for (int i = trim_idx; i < (int)optimized_path.size(); i++) {
                                trimmed.push_back(optimized_path[i]);
                            }
                            optimized_path = trimmed;
                        }
                        // Re-generate reference from trimmed path with current velocity
                        double reference_speed = isxtl? reference_desire_speedxtl : reference_desire_speed;
                        path_smoother->init(optimized_path, start_vel, reference_speed);
                        path_smoother->smoothPath();
                        path_smoother->pathResample();
                        final_path = path_smoother->getPath();
                        if(final_path.size() < 2) return false;
                        reference_path->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, isxtl);
                        reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
                        if(ref_trajectory.size() < 2) return false;
                    }
                }
                return true;
            }
            // (Fix 33a) Check cross-track deviation: if robot is far from planned path,
            // the path is no longer useful — trigger full replan.
            {
                double min_dist = 1e9;
                for (int i = 0; i < (int)optimized_path.size(); i++) {
                    double d = (optimized_path[i].head<2>() - start_point.head<2>()).norm();
                    if (d < min_dist) min_dist = d;
                }
                if (min_dist > 1.0) {
                    ROS_WARN("[Manager REPLAN] robot %.2fm from path, full replan", min_dist);
                    if(!pathFinding(start_point, target_point, start_vel))
                        return false;
                    else
                        return true;
                }
            }
            // Path is collision-free and target unchanged — re-anchor from current position
            // (Fix 14) Always regenerate reference trajectory from robot's current position
            // so the published polynomial starts here, not at the stale original start.
            ROS_INFO("[Manager REPLAN] path safe & target unchanged, re-anchoring from cur pos");
            {
                // Trim optimized_path forward to nearest waypoint ahead of robot
                int closest_idx = 0;
                double min_dist = 1e9;
                for (int i = 0; i < (int)optimized_path.size(); i++) {
                    double d = (optimized_path[i].head<2>() - start_point.head<2>()).norm();
                    if (d < min_dist) {
                        min_dist = d;
                        closest_idx = i;
                    }
                }
                int trim_idx = std::min(closest_idx + 1, (int)optimized_path.size() - 1);
                std::vector<Eigen::Vector3d> trimmed;
                trimmed.push_back(start_point);
                for (int i = trim_idx; i < (int)optimized_path.size(); i++) {
                    trimmed.push_back(optimized_path[i]);
                }
                optimized_path = trimmed;
            }
            {
                double reference_speed = isxtl ? reference_desire_speedxtl : reference_desire_speed;
                path_smoother->init(optimized_path, start_vel, reference_speed);
                path_smoother->smoothPath();
                path_smoother->pathResample();
                final_path = path_smoother->getPath();
                if (final_path.size() < 2) return false;
                reference_path->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, isxtl);
                reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
                if (ref_trajectory.size() < 2) return false;
            }
            return true;
        }

    }
    else {
        if(!pathFinding(start_point, target_point, start_vel))
            return false;
        else
            return true;
    }
}

bool planner_manager::pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                                  const Eigen::Vector3d start_vel)
{
    ROS_WARN("[Manager] start point, (x, y): (%.2f, %.2f)", start_pt.x(), start_pt.y());
    ROS_WARN("[Manager] receive target, (x, y): (%.2f, %.2f)", target_pt.x(), target_pt.y());
    topo_prm->createGraph(start_pt, target_pt);

    optimized_path.clear();
    std::vector<Eigen::Vector3d> origin_path;
    if(topo_prm->min_path.size() > 0){
        origin_path = topo_prm->min_path;
    }else{
        ROS_ERROR("[Manager PLANNING] Invalid target point，global planning failed");
        global_map->resetUsedGrids();
        return false;
    }
    /* 提取出路径（将路径节点都放到一个容器里） */

    /* 二次规划（路径裁减） */
    optimized_path = astar_path_finder->smoothTopoPath(origin_path);  // 剪枝优化topo路径
    std::cout<<"optimized_path size: "<<optimized_path.size()<<std::endl;
    global_map->resetUsedGrids();

    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();

    double reference_speed = isxtl? reference_desire_speedxtl : reference_desire_speed;
    ROS_WARN("[Manager] reference_speed: (%.2f)", reference_speed);

    path_smoother->init(optimized_path, start_vel, reference_speed);
    path_smoother->smoothPath();
    path_smoother->pathResample();
    final_path = path_smoother->getPath();
    ROS_INFO("[Manager] optimizer generate time: %f", (ros::Time::now() - t1).toSec());

    reference_path->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, isxtl);
    reference_path->getRefTrajectory(ref_trajectory, path_smoother->m_trapezoidal_time);
    reference_path->getRefVel();

    astar_path = origin_path;
    if(ref_trajectory.size() < 2){
        return false;
    }
    return true;
}

std::vector<Eigen::Vector3d> planner_manager::localPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt)
{
    std::vector<Eigen::Vector3d> optimized_local_path;
    topo_prm->createLocalGraph(start_pt, target_pt);

    if(topo_prm->min_path.size() > 0){
        optimized_local_path = topo_prm->min_path;
        ROS_DEBUG("[Manager Local] local path find! size is %d", topo_prm->min_path.size());
        return optimized_local_path;
    }else{
        ROS_ERROR("[Manager Local] Invalid target point，global planning failed");
        global_map->resetUsedGrids();
        return optimized_local_path;
    }
}

