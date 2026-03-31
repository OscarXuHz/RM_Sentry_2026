#include "../include/Astar_searcher.h"
#include <time.h>

using namespace std;
using namespace Eigen;


void AstarPathFinder::initGridMap(ros::NodeHandle &nh, double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id, double robot_radius,
                                  double _2d_search_height_low, double _2d_search_height_high, std::shared_ptr<GlobalMap> &_global_map)
{

    global_map = _global_map;
    grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    local_grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("local_grid_map_vis", 1);
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return global_map->gridIndex2coord(global_map->coord2gridIndex(coord));
}


void AstarPathFinder::getCurPositionIndex(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d cur_pos, int &cur_start_id)
{
    /**
 * @brief	获取当前位置在路径中的位置
 */
    cur_start_id = 0;
    double min_dis = 1000000;
    for (int i = 0; i < optimized_path.size() - 1; i++) {

        double path_len = sqrt(pow((optimized_path[i+1].x() - optimized_path[i].x()), 2) + pow((optimized_path[i+1].y() - optimized_path[i].y()), 2));
        int n = static_cast<int>(path_len / (global_map->m_resolution)) + 1;
        double deta_x = (optimized_path[i+1].x() - optimized_path[i].x()) / n;
        double deta_y = (optimized_path[i+1].y() - optimized_path[i].y()) / n;
        for (int j = 1; j < n; j++){
            double temp_x = optimized_path[i].x() + deta_x * i;
            double temp_y = optimized_path[i].y() + deta_y * i;
            double temp_z = 0;
            Vector3d tempPos = {temp_x, temp_y, temp_z};
            double dis = sqrt(pow((cur_pos.x() - tempPos.x()), 2) + pow((cur_pos.y() - tempPos.y()), 2));
            if (dis < min_dis) {
                min_dis = dis;
                cur_start_id = i;
            }
        }
    }
}

bool AstarPathFinder::checkPathCollision(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos, Eigen::Vector3d cur_pos,
                                         Eigen::Vector3d &collision_start_point, Eigen::Vector3d &collision_target_point, int &path_start_id, int &path_end_id)
{
    /**
     * @brief	检查路径是否与障碍物碰撞，并返回碰撞点坐标  0对应起点
     */
    int cur_start_id = 0;
    getCurPositionIndex(optimized_path, cur_pos, cur_start_id);
    int path_start_count = cur_start_id;
    int path_end_count = cur_start_id + 1;
    Vector3d headPos, tailPos;

    /* 取出第一个路径节点作为第一个尾节点，此后tailPos记录的就一直是检查碰撞的上一个节点 */
    for(int i = cur_start_id ; i < optimized_path.size() - 1; i++){
        headPos = optimized_path[i];
        tailPos = optimized_path[i+1];
        double path_len = sqrt(pow((tailPos.x() - headPos.x()), 2) + pow((tailPos.y() - headPos.y()), 2));
        int n = static_cast<int>(path_len / (global_map->m_resolution)) + 1;
        double deta_x = (headPos.x() - tailPos.x()) / n;
        double deta_y = (headPos.y() - tailPos.y()) / n;
        /* 如果连线上存在某个点落在障碍物栅格中则认为连线经过障碍物 */
        for (int j = 1; j < n; j++) {
            double temp_x = tailPos.x() + deta_x * j;
            double temp_y = tailPos.y() + deta_y * j;
            double temp_z = 0;
            Vector3d tempPos = {temp_x, temp_y, temp_z};

            /* 用空间点坐标找到其所对应的栅格 */
            Vector3i tempID = global_map->coord2gridIndex(tempPos);
            /* 如果两点连线经过障碍物栅格，则把返回true */
            if (global_map->isOccupied(tempID, false))
            {
                collision_pos = tempPos;
                ROS_INFO("[A Star] find collision pos: %f %f", collision_pos.x(), collision_pos.y());
                collision_start_point = cur_pos;  // 碰撞路段起点
                path_start_id = cur_start_id;
                while(i<optimized_path.size() - 1){
                    i++;
                    tailPos = optimized_path[i];
                    ROS_INFO("[A Star] find start pos: %f %f", headPos.x(), headPos.y());
                    Vector3i headtempID = global_map->coord2gridIndex(tailPos);
                    if(global_map->isFree(headtempID)){
                        collision_target_point = tailPos;  // 碰撞路段终点
                        path_end_id = i;
                        break;
                    }
                }
                return true;
            }
        }

    }
    return false;
}

std::vector<Eigen::Vector3d> AstarPathFinder::smoothTopoPath(std::vector<Eigen::Vector3d> topo_path)
{
    if(topo_path.size() < 2)
    {
        return topo_path;
    }
    // ── DIAGNOSTIC: log input topo path ──
    ROS_WARN("[smoothTopoPath] INPUT size=%zu", topo_path.size());
    for (size_t k = 0; k < topo_path.size(); k++) {
        ROS_WARN("[smoothTopoPath]   in[%zu]: (%.3f, %.3f, %.3f)", k,
                 topo_path[k].x(), topo_path[k].y(), topo_path[k].z());
    }
    // Test direct start→end visibility
    {
        Eigen::Vector3d cp;
        bool dv = lineVisib(topo_path.back(), topo_path.front(), cp, 0.05, 2);
        ROS_WARN("[smoothTopoPath] lineVisib(start→end): %s  colli=(%.2f,%.2f)",
                 dv ? "TRUE(!)" : "false", cp.x(), cp.y());
    }
    Eigen::Vector3d tail_pos, colli_pt;
    std::vector<Eigen::Vector3d> smooth_path;
    Eigen::Vector3d head_pos = topo_path[0];
    smooth_path.push_back(head_pos);
    int iter_idx = 0;
    bool collision = true;
    double last_height = head_pos.z();  // 初始化高度为起点高度，防止起点在桥洞区域
    int iter_count = 0;
    Eigen::Vector3d prev_head_pos = head_pos;  // (Fix 53d) Track previous head for stuck detection
    int stuck_count = 0;

    while(collision)  // 如果迭代到最后一个点不出现碰撞，则认为可以直接把当前点连接到终点，整体优化完毕
    {
        iter_count++;
        if(iter_count > 500){
            // (Fix 47b) Reduced from 1000; if still looping after 500 iterations
            // the pruning is stuck.  Return the partial smooth path accumulated so
            // far plus the remaining raw topo path from the current head position.
            ROS_WARN("[smoothTopoPath] iteration limit (%d), returning partial smooth + raw tail", iter_count);
            break;
        }
        collision = false;
        Eigen::Vector3d temp;
        for(int i = iter_idx; i <topo_path.size() - 1; i++)  // 找topo路径的每一段
        {
            last_height = topo_path[i].z();

//            std::cout<<"topo_path[i].x(): "<<topo_path[i].x()<<"topo_path[i].y()"<<topo_path[i].y()<<std::endl;

            double x_offset = topo_path[i + 1].x() - topo_path[i].x();
            double y_offset = topo_path[i + 1].y() - topo_path[i].y();

            double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

            int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / 0.05;  // 在每一段采样
            for (int j = 1; j <= n; j++)
            {
                bool second_height = false;
                tail_pos.x() = topo_path[i].x() + j * 0.05 * x_offset / distance;
                tail_pos.y() = topo_path[i].y() + j * 0.05 * y_offset / distance;
                Eigen::Vector3i temp_id = global_map->coord2gridIndex(tail_pos);
                if(!global_map->GridNodeMap[temp_id.x()][temp_id.y()]->exist_second_height){
                    tail_pos.z() = global_map->GridNodeMap[temp_id.x()][temp_id.y()]->height;
                    last_height = tail_pos.z();
                }else{
                    tail_pos.z() = last_height;  //进入桥洞区域后高度等于last height
                    if(tail_pos.z() < 0.4){
                        second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
                    }
                }

                // (Fix 54c) Use isStaticOccupied — consistent with topo search (Fix 53a).
                // Live lidar obstacles caused detour loops in smoothing.
                if(global_map->isStaticOccupied(temp_id.x(), temp_id.y(), second_height)){
                    // (Fix 16) Occupied waypoint on topo path — treat as collision to
                    // prevent visibility shortcutting from creating paths that graze obstacles
                    if(!collision){
                        collision = true;
                        temp = tail_pos;
                        bool easy = getNearPoint(temp, head_pos, temp);
                        // (Fix 53b) Fallback: if getNearPoint produced NaN, use nearest topo node
                        if (std::isnan(temp.x()) || std::isnan(temp.y())) {
                            temp = topo_path[std::min(i + 1, (int)topo_path.size() - 1)];
                        }
                        iter_idx = i;
                    }
                    continue;
                }

                // (Fix 49c) Reduced cell_margin from 3→2 (0.10m) to allow more direct paths
                // while still maintaining safety beyond the 0.35m inflation.
                if (!lineVisib(tail_pos, head_pos, colli_pt, 0.05, 2) && !collision){  // 记录最近的可见点的第一个不可见点
                    collision = true;
                    Eigen::Vector3i temp_id = global_map->coord2gridIndex(tail_pos);
                    temp = {topo_path[i].x() + (j) * 0.05 * x_offset / distance,
                            topo_path[i].y() + (j) * 0.05 * y_offset / distance,
                            last_height};
                    temp = tail_pos;
                    bool easy = getNearPoint(temp, head_pos, temp);
                    // (Fix 53b) Fallback: if getNearPoint produced NaN, use nearest topo node
                    if (std::isnan(temp.x()) || std::isnan(temp.y())) {
                        temp = topo_path[std::min(i + 1, (int)topo_path.size() - 1)];
                    }
                    iter_idx = i;
                }
                else if (lineVisib(tail_pos, head_pos, colli_pt, 0.05, 2)){
                    // 这里的意思是，只要最后一段可见，就不用再检查了，直接连。即为连接最后一个可见的topo点
                    collision = false;
                }
            }
        }
        if(collision){
            // (Fix 53d) Stuck detection: if head_pos barely moved, skip to next topo waypoint
            double head_shift = (temp.head<2>() - prev_head_pos.head<2>()).norm();
            if (head_shift < 0.05) {
                stuck_count++;
                if (stuck_count >= 1) {
                    // (Fix 53e) Skip to the endpoint of the current topo segment.
                    // Previously used topo_path[skip+1] which jumped 2 nodes ahead,
                    // losing critical intermediate waypoints around walls.
                    int next_wp = std::min(iter_idx + 1, (int)topo_path.size() - 1);
                    temp = topo_path[next_wp];
                    iter_idx = next_wp;  // next iteration starts from this segment
                    stuck_count = 0;
                    ROS_WARN("[smoothTopoPath] stuck detected, skipping to topo[%d]", next_wp);
                }
            } else {
                stuck_count = 0;
            }
            prev_head_pos = head_pos;
            head_pos = temp;
//            last_height = head_pos.z();
            smooth_path.push_back(temp);
        }else{
            smooth_path.push_back(topo_path.back());
            break;
        }

    }
    // (Fix 53e) Dedup: remove consecutive near-duplicate points (<0.1m apart)
    auto dedup = [](std::vector<Eigen::Vector3d>& path) {
        if (path.size() < 3) return;
        std::vector<Eigen::Vector3d> clean;
        clean.push_back(path.front());
        for (size_t i = 1; i < path.size() - 1; i++) {
            if ((path[i].head<2>() - clean.back().head<2>()).norm() > 0.1)
                clean.push_back(path[i]);
        }
        clean.push_back(path.back()); // always keep end
        path = clean;
    };

    if(iter_count > 500){
        // (Fix 47b) Build a safe fallback: the partial smooth_path we have so far
        // plus the remaining raw topo nodes from the current head_pos onward.
        if (smooth_path.size() >= 2) {
            double min_d = 1e9;
            int best_idx = (int)topo_path.size() - 1;
            for (int k = 0; k < (int)topo_path.size(); k++) {
                double d = (topo_path[k].head<2>() - head_pos.head<2>()).norm();
                if (d < min_d) { min_d = d; best_idx = k; }
            }
            for (int k = best_idx + 1; k < (int)topo_path.size(); k++) {
                smooth_path.push_back(topo_path[k]);
            }
            dedup(smooth_path);
            ROS_WARN("[smoothTopoPath] fallback: %zu pts after dedup", smooth_path.size());
            return smooth_path;
        }
        return topo_path;
    }
    dedup(smooth_path);
    ROS_WARN("[smoothTopoPath] OUTPUT size=%zu, iter_count=%d", smooth_path.size(), iter_count);
    for (size_t k = 0; k < smooth_path.size(); k++) {
        ROS_WARN("[smoothTopoPath]   out[%zu]: (%.3f, %.3f, %.3f)", k,
                 smooth_path[k].x(), smooth_path[k].y(), smooth_path[k].z());
    }
    return smooth_path;

}

bool AstarPathFinder::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& colli_pt, double thresh, int cell_margin)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z, p1_x, p1_y, p1_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    p1_x = p1.x();
    p1_y = p1.y();
    p1_z = p1.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();
    double z_offset = p1.z() - p2.z();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = int(double(std::sqrt(pow(x_offset, 2) + pow(y_offset, 2))) / double(thresh));
    int idx, idy, idz, idx_end, idy_end, idz_end;
    global_map->coord2gridIndex(p2_x, p2_y, p2_z, idx, idy, idz);
    global_map->coord2gridIndex(p1_x, p1_y, p1_z, idx_end, idy_end, idz_end);
    double last_height = p2_z;  ///起点高度

    for(int i = 0; i<n+1; i++)
    {
        if(i == n){
            ray_ptx = p1_x;
            ray_pty = p1_y;
            ray_ptz = p1_z;
        }else {
            ray_ptx = p2_x + i * thresh * x_offset / distance;
            ray_pty = p2_y + i * thresh * y_offset / distance;
            ray_ptz = 0.0;
        }
//        std::cout<<"ray_ptx: "<<ray_ptx<<" ray_pty: "<<ray_pty<<std::endl;

        int pt_idx, pt_idy, pt_idz;
        global_map->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height;
        bool second_height = false;
        if(!global_map->GridNodeMap[pt_idx][pt_idy]->exist_second_height) {
            height = global_map->getHeight(pt_idx, pt_idy);  //非桥洞区域正常通行
        }else{
            height = last_height;
            if(global_map->GridNodeMap[idx_end][idy_end]->exist_second_height){
                last_height = p1_z;
            }

            if(height < 0.4){
                second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
            }
        }


        // (Fix 54c) Use isStaticOccupied — consistent with topo search (Fix 53a).
        if (global_map->isStaticOccupied(pt_idx, pt_idy, second_height)){
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = global_map->gridIndex2coord(temp_idx);
            return false;
        }
        // (Fix 47a) Safety margin: check cells at ±cell_margin offsets in cardinal
        // directions.  This ensures the shortcut maintains clearance from obstacles
        // beyond the base inflation radius, preventing near-miss pruning.
        // (Fix 54c) All margin checks also use isStaticOccupied.
        if (cell_margin > 0) {
            bool margin_hit = false;
            for (int m = 1; m <= cell_margin && !margin_hit; m++) {
                int mx, my;
                mx = pt_idx + m; my = pt_idy;
                if (mx < global_map->GLX_SIZE && global_map->isStaticOccupied(mx, my, second_height)) { margin_hit = true; break; }
                mx = pt_idx - m;
                if (mx >= 0 && global_map->isStaticOccupied(mx, my, second_height)) { margin_hit = true; break; }
                mx = pt_idx; my = pt_idy + m;
                if (my < global_map->GLY_SIZE && global_map->isStaticOccupied(mx, my, second_height)) { margin_hit = true; break; }
                my = pt_idy - m;
                if (my >= 0 && global_map->isStaticOccupied(mx, my, second_height)) { margin_hit = true; break; }
            }
            if (margin_hit) {
                Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
                colli_pt = global_map->gridIndex2coord(temp_idx);
                return false;
            }
        }
        if (height - last_height >= 0.12)
        {
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = global_map->gridIndex2coord(temp_idx);
            return false;
        }
        else if (height - last_height <= -0.3)
        {
            Eigen::Vector3i temp_idx = {pt_idx, pt_idy, pt_idz};
            colli_pt = global_map->gridIndex2coord(temp_idx);
            return false;
        }
        last_height = height;
    }
    return true;
}



bool AstarPathFinder::getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point)
{
    double check_distance = 0.3;
    while(check_distance >= 0.1)
    {
        double delta_x = headPos.x() - tailPos.x();
        double delta_y = headPos.y() - tailPos.y();
        // (Fix 53b) Guard against division by zero when headPos ≈ tailPos.
        // This caused NaN coordinates that poisoned the entire downstream
        // pipeline (MINCO → reference_path → straight-line trajectory).
        double norm = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (norm < 1e-6) {
            best_point = headPos;
            return false;
        }
        Eigen::Vector3d collision_pt;

        Eigen::Vector3d headPos_left, headPos_right;
        headPos_left.x() = headPos.x() + check_distance * delta_y / norm;
        headPos_left.y() = headPos.y() - check_distance * delta_x / norm;
        headPos_left.z() = headPos.z();

        headPos_right.x() = headPos.x() - check_distance * delta_y / norm;
        headPos_right.y() = headPos.y() + check_distance * delta_x / norm;
        headPos_right.z() = headPos.z();

        Vector3i tempID_left = global_map->coord2gridIndex(headPos_left);
        Vector3i tempID_right = global_map->coord2gridIndex(headPos_right);

        int check_swell = (check_distance / 0.1) -1;
        if (lineVisib(tailPos, headPos_left, collision_pt, 0.1))
        {
            best_point = headPos_left;
            return true;
        }else if (lineVisib(tailPos, headPos_right, collision_pt, 0.1)){
            best_point = headPos_right;
            return true;
        }else{
            best_point = headPos;
        }

        check_distance -= 0.1;
    }

    return false;
}

bool AstarPathFinder::checkPointCollision(Eigen::Vector3i path_point, int check_swell)
{
    Eigen::Vector3i path_succ;
    for (int idx = -check_swell; idx <= check_swell; idx++)
    {
        for (int jdx = -check_swell; jdx <= check_swell; jdx++)
        {
            path_succ.x() = path_point.x() + idx;
            path_succ.y() = path_point.y() + jdx;
            path_succ.z() = path_point.z();
            // (Fix 54c) Use isStaticOccupied for collision swell check
            if (global_map->isStaticOccupied(path_succ, false))
            {
                return true;
            }
        }
    }
    return false;
}

bool AstarPathFinder::findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num)
{
    Eigen::Vector3i path_succ;
    std::vector<double> path_distance;
    std::vector<Eigen::Vector3i> path_free;
    for (int idx = -check_num; idx <= check_num; idx++)
    {
        for (int jdx = -check_num; jdx <= check_num; jdx++)
        {
            path_succ.x() = path_point.x() + idx;
            path_succ.y() = path_point.y() + jdx;
            path_succ.z() = path_point.z();
            if (global_map->isFree(path_succ))
            {
                path_free.push_back(path_succ);
                double distance = sqrt(pow(path_succ.x() - path_point.x(), 2) + pow(path_succ.y() - path_point.y(), 2));
                path_distance.push_back(distance);
                // output_point = path_succ;
                // return true;
            }
        }
    }
    if (path_free.empty())
    {
        return false;
    }
    std::vector<double>::iterator maxdistance = std::max_element(path_distance.begin(), path_distance.end());
    int index = std::distance(path_distance.begin(), maxdistance);
    output_point = path_free[index];
    return true;
}


/**
 * @brief	用点云坐标生成栅格中心点坐标（相当于降采样）后发给rviz以显示出来
 *          （二维规划下只取指定高度范围的点云）
 * @param
 */
void AstarPathFinder::visGridMap() {
//    ROS_WARN("start publish GridMap");
    pcl::PointCloud <pcl::PointXYZRGB> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    pcl::PointXYZRGB pt;

    for (int i = 0; i < global_map->GLX_SIZE; i++) {
        for (int j = 0; j < global_map->GLY_SIZE; j++) {
            for (int k = 0; k < global_map->GLZ_SIZE; k++) {
                Eigen::Vector3i temp_grid(i, j, 0);

                if (global_map->isOccupied(temp_grid, false))
                {
                    Eigen::Vector3d temp_pt = global_map->gridIndex2coord(temp_grid);
                    pt.x = temp_pt(0);
                    pt.y = temp_pt(1);
                    pt.z = 0.0;
                    pt.r = 255.0;
                    pt.g = 255.0;
                    pt.b = 255.0;
                    cloud_vis.points.push_back(pt);

                }
                if(global_map->GridNodeMap[i][j]->visibility > 0.1)
                {
                    Eigen::Vector3d temp_pt = global_map->gridIndex2coord(temp_grid);
                    pt.x = temp_pt(0);
                    pt.y = temp_pt(1);
                    pt.z = 0.0;
                    pt.r = 255 - global_map->GridNodeMap[i][j]->visibility * 12.0;
                    pt.g = 0.0;
                    pt.b = global_map->GridNodeMap[i][j]->visibility * 12.0;
                    cloud_vis.points.push_back(pt);
                }

            }
        }
    }

    /* 将障碍物的PCL点云数据类型转换为ROS点云通信数据类型*/
    pcl::toROSMsg(cloud_vis, map_vis);

    /* 设置坐标系的名称 */
    map_vis.header.frame_id = "map";

    /* 发布障碍物点云到rviz中 */
    grid_map_vis_pub.publish(map_vis);
    int temp = 1;
    while (temp--) {
        pcl::toROSMsg(cloud_vis, map_vis);
        map_vis.header.frame_id = "map";
        grid_map_vis_pub.publish(map_vis);
    }
}

/**
 * @brief	用点云坐标生成栅格中心点坐标（相当于降采样）后发给rviz以显示出来
 *          （二维规划下只取指定高度范围的点云）
 * @param
 */
void AstarPathFinder::visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag) {
    pcl::PointCloud <pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    pcl::PointXYZ pt;
    if (!swell_flag) {
        for (int idx = 0; idx < (int) cloud.points.size(); idx++) {

            pt = cloud.points[idx];
            Eigen::Vector3d cor_round = coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = cor_round(2);
            cloud_vis.points.push_back(pt);
        }
    } else {
        for (int i = 0; i < global_map->GLX_SIZE; i++) {
            for (int j = 0; j < global_map->GLY_SIZE; j++) {
                for (int k = 0; k < global_map->GLZ_SIZE; k++) {
                    Eigen::Vector3i temp_grid(i, j, k);
                    if (global_map->isLocalOccupied(temp_grid)) {
                        Eigen::Vector3d temp_pt = global_map->gridIndex2coord(temp_grid);
                        pt.x = temp_pt(0);
                        pt.y = temp_pt(1);
                        pt.z = global_map->getHeight(i, j) * 1;;
                        cloud_vis.points.push_back(pt);
                    }

                    if(global_map->GridNodeMap[i][j]->exist_second_height == true && global_map->GridNodeMap[i][j]->second_local_occupancy == true)
                    {
                        Eigen::Vector3d temp_pt = global_map->gridIndex2coord(temp_grid);
                        pt.x = temp_pt(0);
                        pt.y = temp_pt(1);
                        pt.z = 2.0;
                        cloud_vis.points.push_back(pt);
                    }
                }
            }
        }
    }



    /* 设置点云的规则（无序、坐标值有限） */
    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;
//    ROS_WARN("local point size is  %d ", (int)cloud_vis.points.size());

    /* 将障碍物的PCL点云数据类型转换为ROS点云通信数据类型*/
    pcl::toROSMsg(cloud_vis, map_vis);

    /* 设置坐标系的名称 */
    map_vis.header.frame_id = "map";

    /* 发布障碍物点云到rviz中 */
    local_grid_map_vis_pub.publish(map_vis);
    pcl::toROSMsg(cloud_vis, map_vis);
    map_vis.header.frame_id = "map";
}
