//
// Created by hitcrt on 2023/5/4.
//

#include "../include/RM_GridMap.h"

void GlobalMap::initGridMap(ros::NodeHandle &nh, cv::Mat occ_map, std::string bev_map_file, std::string distance_map_file,
                            double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                            int max_x_id, int max_y_id, int max_z_id, double robot_radius,
                            double _2d_search_height_low, double _2d_search_height_high, double _search_radius)
{
    m_node = nh;

    /* 设置地图的边界与尺寸 */
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    nh.param("tracking_node/height_bias", m_height_bias, -0.5);
    nh.param("tracking_node/height_interval", m_height_interval, 2.0);
    nh.param("tracking_node/height_threshold", m_height_threshold, 0.1);
    nh.param("tracking_node/height_sencond_high_threshold", m_height_sencond_high_threshold, 0.4);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
    m_resolution = _resolution;
    m_inv_resolution = 1.0 / _resolution;
    data = new uint8_t[GLXY_SIZE];
    l_data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    memset(l_data, 0, GLXY_SIZE * sizeof(uint8_t));
    /* 设置2D搜索高度 */
    m_2d_search_height_low = _2d_search_height_low;
    m_2d_search_height_high = _2d_search_height_high;
    /* 设置机器人碰撞检测半径 */
    m_robot_radius = robot_radius;
    m_robot_radius_dash = robot_radius;
    search_radius = _search_radius;


    cv::Mat bev_map;
    bev_map = cv::imread(bev_map_file);

    // 四个回调函数分别对应真实与仿真环境的回调数据
    m_local_point_sub = m_node.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_world", 1, &GlobalMap::lidarCloudCallback, this);
    gazebo_point_sub = m_node.subscribe<sensor_msgs::PointCloud2>("/mbot/velodyne_points", 1, &GlobalMap::gazeboCloudCallback, this);

//    global_map_pub = m_node.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    int swell_num = (int) (m_robot_radius / m_resolution) * 2;
    int GL_SIZE = swell_num + (int)(search_radius/m_resolution);

    GridNodeMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++){
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }

    GridNodeLocalMap = new GridNodePtr *[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++){
        GridNodeLocalMap[i] = new GridNodePtr[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++){
            Eigen::Vector3i tmpIdx(i, j, 0);
            Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeLocalMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }

    occ_map = swellOccMap(occ_map);
    bevMapToHeight(bev_map, occ_map);
    processSecondHeights();

    occMaptoObstacle(occ_map);
    computeNearStaticMap();  // Fix 61a: precompute dilated static map
    m_local_cloud.reset(new pcl::PointCloud <pcl::PointXYZ>);
}

void GlobalMap::gazeboCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point)
{
//    ROS_WARN("start gazebo pointcloud callback");
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();
    m_local_cloud->clear();

    pcl::fromROSMsg(*point, *local_cloud_3d);

    Eigen::Matrix3d R_x; // 计算旋转矩阵的X分量
    R_x << 1, 0, 0,
            0, cos(odom_posture[0]), -sin(odom_posture[0]),
            0, sin(odom_posture[0]), cos(odom_posture[0]);

    Eigen::Matrix3d R_y; // 计算旋转矩阵的Y分量
    R_y << cos(odom_posture[1]), 0, sin(odom_posture[1]),
            0, 1, 0,
            -sin(odom_posture[1]), 0, cos(odom_posture[1]);

    Eigen::Matrix3d R_z; // 计算旋转矩阵的Z分量
    R_z << cos(odom_posture[2]), -sin(odom_posture[2]), 0,
            sin(odom_posture[2]), cos(odom_posture[2]), 0,
            0, 0, 1;
    Eigen::Matrix3d R = R_z * R_y * R_x;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = odom_position;

    // 定义输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 进行坐标系转换
    pcl::transformPointCloud(*local_cloud_3d, *output_cloud, transform);

    clock_t time_1 = clock();

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    int point_size = 0;

    for (int i = 0; i < (int) output_cloud->points.size(); i++){
        pt_3d = output_cloud->points[i];
        /// 在范围内的点云bevMapToHeight
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < search_radius) && (abs(pt_3d.y - odom_position(1)) < search_radius)){
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z + 0.2;
            m_local_cloud->points.push_back(pt);
            point_size++;
        }
    }

    clock_t time_2 = clock();
//    ROS_WARN("Time in local point  is %lf ms", (double)(time_2 - time_1) * 1000 / CLOCKS_PER_SEC);
    localPointCloudToObstacle(*m_local_cloud, true, odom_position);

//    static std::queue<int> old_cloud_num;
//    static int record_times = 0;
//    old_cloud_num.push(point_size);
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());
//    if (++record_times > 6)
//    {
//        int abandon_num = old_cloud_num.front();
//        for (int i = 0; i < abandon_num; i++)
//        {
//            m_local_cloud->points.pop_back();
//        }
//        old_cloud_num.pop();
//    }
//    reverse(m_local_cloud->points.begin(), m_local_cloud->points.end());
}

void GlobalMap::lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point)
{
    static pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud_3d(new pcl::PointCloud<pcl::PointXYZ>);
    local_cloud_3d->clear();
    m_local_cloud->clear();

    pcl::fromROSMsg(*point, *local_cloud_3d);

    /* 点云区域筛选 */
    pcl::PointXYZ pt;
    pcl::PointXYZ pt_3d;
    for (int i = 0; i < (int)local_cloud_3d->points.size(); i++)
    {
        pt_3d = local_cloud_3d->points[i];
        if (pt_3d.z > m_2d_search_height_low && pt_3d.z < m_2d_search_height_high && (abs(pt_3d.x - odom_position(0)) < search_radius) && (abs(pt_3d.y - odom_position(1)) < search_radius))
        {
            pt.x = pt_3d.x;
            pt.y = pt_3d.y;
            pt.z = pt_3d.z;
            m_local_cloud->points.push_back(pt);
        }
    }

    localPointCloudToObstacle(*m_local_cloud, true, odom_position);
}


cv::Mat GlobalMap::swellOccMap(cv::Mat occ_map)
{
    cv::Mat occ = cv::Mat::zeros(occ_map.rows, occ_map.cols, CV_8UC1);
    int swell_num = (int) (m_robot_radius / m_resolution);

    for (int i = 0; i < occ_map.rows; i++) {
        for (int j = 0; j < occ_map.cols; j++) {
            if (occ_map.at<uchar>(i, j) > 10)
            {   // 膨胀直接画圆
                cv::circle(occ, cv::Point(j, i), swell_num, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    ROS_WARN("map swell done");
    return occ;
}

void GlobalMap::bevMapToHeight(const cv::Mat bev_map, const cv::Mat occ_map)
{
    /// 高度地图建立！
    std::vector<cv::Mat> channels;
    cv::split(bev_map, channels);
    cv::Mat bev = channels.at(0);

    std::vector<cv::Mat> channels_occ;
    cv::split(occ_map, channels_occ);
    cv::Mat occ = channels_occ.at(0);

    pcl::PointXYZ pt;
    for (int i = 1; i < bev.rows -1 ; i++){
        for (int j = 1; j < bev.cols - 1; j++){
            bool exist_heights = false;;
            pt.x = (j + 0.5) * m_resolution + gl_xl;
            pt.y = (bev.rows - i - 0.5) * m_resolution + gl_yl;
            pt.z = (bev.at<uchar>(i, j) * m_height_interval / 255) + m_height_bias; /// TODO map param

            if (occ.at<uchar>(i, j) == 0) {
                for (int idx = -1; idx <= 1; idx++) {
                    for (int idy = -1; idy <= 1; idy++) {
                        if (occ.at<uchar>(i+idx, j+idy) != 0) continue;  // skip occupied neighbors — wall edges are not bridges
                        double temp_z = (bev.at<uchar>(i+idx, j+idy) * m_height_interval / 255) + m_height_bias;
                        if(abs(temp_z - pt.z) > 0.4 && std::max(temp_z, (double)pt.z) > 0.5){  // require meaningful elevation for genuine bridge
                            exist_heights = true;
                            second_heights_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
                        }
                    }
                }
            }
            setHeight(pt.x, pt.y, pt.z, exist_heights);  // 如果高度变化太大的可通行区域点视为桥洞出入口
        }
    }
    std::cout<<"second_heights_points: "<<second_heights_points.size()<<std::endl;
    ROS_WARN("[Map Build] bev build done");
}

void GlobalMap::occMaptoObstacle(const cv::Mat occ_map)
{
    pcl::PointXYZ pt;
    ROS_WARN("R.rows %d R.cols %d", occ_map.rows, occ_map.cols);

    for (int i = 0; i < occ_map.rows; i++)
    {
        for (int j = 0; j < occ_map.cols; j++)
        {
            if (occ_map.at<uchar>(i, j) != 0)
            {
                pt.x = (j+0.5) * m_resolution + gl_xl;
                pt.y = (occ_map.rows - i - 0.5) * m_resolution + gl_yl;
                setObs(pt.x, pt.y);
            }
        }
    }
}

void GlobalMap::processSecondHeights()
{  // 处理第二高度的地图，处理方式与之前相同，根据高度变化剧烈的位置栅格进行处理
    while(!second_heights_points.empty()){
        Eigen::Vector3d temp_point = second_heights_points.back();
        double x_max = temp_point.x();
        double x_min = temp_point.x();
        double y_max = temp_point.y();
        double y_min = temp_point.y();
        second_heights_points.pop_back();
        for(std::vector<Eigen::Vector3d>::iterator it = second_heights_points.begin(); it != second_heights_points.end(); it++){
            if(std::sqrt((temp_point.x() - (*it).x()) * (temp_point.x() - (*it).x())
                         + (temp_point.y() - (*it).y()) * (temp_point.y() - (*it).y())) < 4.0){
                x_max = std::max(x_max, (*it).x());
                x_min = std::min(x_min, (*it).x());
                y_max = std::max(y_max, (*it).y());
                y_min = std::min(y_min, (*it).y());
                it = second_heights_points.erase(it);
                it--;
            }
        }
        std::cout<<"xmax: "<<x_max<<" xmin: "<<x_min<<" ymax: "<<y_max<<" ymin: "<<y_min<<std::endl;

        int idx_x_max = std::min(static_cast<int>((x_max - gl_xl + 0.01) * m_inv_resolution), GLX_SIZE - 1);
        int idx_y_max = std::min(static_cast<int>((y_max - gl_yl + 0.01) * m_inv_resolution), GLY_SIZE - 1);
        int idx_x_min = std::max(static_cast<int>((x_min - gl_xl + 0.01) * m_inv_resolution), 0);
        int idx_y_min = std::max(static_cast<int>((y_min - gl_yl + 0.01) * m_inv_resolution), 0);
        for(int i = idx_x_min; i<=idx_x_max; i++) {
            for (int j = idx_y_min; j <= idx_y_max; j++) {
                GridNodeMap[i][j]->second_local_swell=true;
                for(int k = -2; k <= 2; k++){
                    for(int m = -2; m <= 2; m++){
                        int ni = i + k, nj = j + m;
                        if(ni >= 0 && ni < GLX_SIZE && nj >= 0 && nj < GLY_SIZE)
                            GridNodeMap[ni][nj]->exist_second_height = true;
                    }
                }

            }
        }
    }

}


void GlobalMap::pointCloudToObstacle()
{

}

/**
 * @brief	利用点云坐标为栅格地图设置障碍物
 *          （二维规划下只取指定高度范围的点云生成一层障碍物）
 * @param
 */
void GlobalMap::localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &swell_flag, const Eigen::Vector3d current_position, const int raycast_num)
{
    current_position_index = coord2gridIndex(current_position);  // 当前位置的栅格坐标

    pcl::PointXYZ pt;
    // ── Temporal decay instead of hard reset ──────────────────────
    // Keep obstacles alive for OBS_PERSIST_FRAMES scan cycles so that
    // objects behind the robot (no longer in the current scan) remain
    // on the map for ~0.5 s at 10 Hz.  This fixes MPC path flickering
    // caused by obstacle set changing completely every frame.
    static const uint8_t OBS_PERSIST_FRAMES = 5;
    for (int k = 0; k < GLXY_SIZE; ++k) {
        if (l_data[k] > 0) l_data[k]--;
    }
    // ROS_WARN("m_robot_radius_dash: %f", m_robot_radius_dash);

    if (!swell_flag){
        ROS_ERROR("ERROR");
        for (int idx = 0; idx < (int)cloud.points.size(); idx++)
        {
            pt = cloud.points[idx];
            localSetObs(pt.x, pt.y, pt.z);
        }
    }
    else{
        /* 膨胀处理 */
        for (int idx = 0; idx < (int)cloud.points.size(); idx++)
        {
            pt = cloud.points[idx];
            int swell_num = (int)(m_robot_radius_dash / m_resolution);
//            int swell_num = 1;

            if (pt.x < gl_xl || pt.y < gl_yl || pt.z < gl_zl ||
                pt.x >= gl_xu || pt.y >= gl_yu || pt.z >= gl_zu)
                continue;

            int idx_x = static_cast<int>((pt.x - gl_xl) * m_inv_resolution);
            int idx_y = static_cast<int>((pt.y - gl_yl) * m_inv_resolution);
            int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);
            /// 凡是高度差距超过阈值m_height_threshold或者过高的，以及桥洞或者已经是被占据的栅格都不考虑
            if (GridNodeMap[idx_x][idx_y]->height + m_height_threshold > pt.z || GridNodeMap[idx_x][idx_y]->height + 1.0 < pt.z)
                continue;
            ///底层tracking中我们不对桥洞的障碍物进行过多处理，不考虑桥洞中的障碍物，简化一下逻辑
            // FIX: Only skip STATIC walls (data==1) and bridge areas.
            // Previously used isOccupied() which also skipped cells with l_data>0,
            // preventing the persist counter from being refreshed → obstacles
            // flickered off every 5 frames (~0.5s gap) even while continuously
            // detected by the LiDAR.
            if(data[idx_x * GLY_SIZE + idx_y] == 1 || GridNodeMap[idx_x][idx_y]->exist_second_height){
                continue;
            }

            /// 通过高度地图阈值筛选之后膨胀设计全局地图的占用栅格数据，注意我们不需要设置局部地图高度
            for (int i = -swell_num; i <= swell_num; i++)
            {
                for (int j = -swell_num; j <= swell_num; j++)
                {
                    double temp_x = i * m_resolution + pt.x;
                    double temp_y = j * m_resolution + pt.y;
                    double temp_z = pt.z;
                    /* 利用点云坐标为栅格地图设置障碍物 */
                    localSetObs(temp_x, temp_y, temp_z);
                }
            }
        }
    }
}

/**
 * @brief	设置障碍物（将障碍物信息保存到路径规划器里）
 *          膨胀处理（防撞设计）
 * @param   组成障碍物空间点坐标
 */
void GlobalMap::setObs(const double coord_x, const double coord_y)
{
    if (coord_x < gl_xl || coord_y < gl_yl  ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);

    /* 将障碍物映射为容器里值为1的元素 */
    data[idx_x * GLY_SIZE + idx_y] = 1;
}


/**
 * @brief	设置局部障碍物（将障碍物信息保存到路径规划器里）
 *          膨胀处理（防撞设计）
 * @param   组成障碍物空间点坐标
 */
void GlobalMap::localSetObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;
    // ROS_WARN("gl_zl %f", gl_zl);

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = static_cast<int>((coord_x - gl_xl) * m_inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * m_inv_resolution);
    int idx_z = static_cast<int>((0 - gl_zl) * m_inv_resolution);

    /* 将障碍物映射为容器里值为持久计数器,根据这里的值做raycast process */
    static const uint8_t OBS_PERSIST = 5;  // match decay constant
    l_data[idx_x * GLY_SIZE + idx_y] = OBS_PERSIST;
}



void GlobalMap::setRadiusDash(const double dash_radius)
{
    m_robot_radius_dash = dash_radius;
}


void GlobalMap::setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights)
{
    if (coord_x < gl_xl || coord_y < gl_yl ||
        coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    /* 用障碍物空间点坐标找到其所对应的栅格 */
    int idx_x = std::min(static_cast<int>((coord_x - gl_xl + 0.01) * m_inv_resolution), GLX_SIZE - 1);
    int idx_y = std::min(static_cast<int>((coord_y - gl_yl + 0.01) * m_inv_resolution), GLY_SIZE - 1);

    GridNodeMap[idx_x][idx_y]->height = coord_z;
}


double GlobalMap::getHeight(const int idx_x, const int idx_y)
{
     if (idx_x >= GLX_SIZE || idx_y >= GLY_SIZE || idx_x < 0 || idx_y < 0)
         return 0.0;
     return GridNodeMap[idx_x][idx_y]->height;
}

void GlobalMap::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid(GridNodeMap[i][j]);
        }
}


Eigen::Vector3d GlobalMap::gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * m_resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * m_resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * m_resolution + gl_zl;

    return pt;
}

Eigen::Vector3i GlobalMap::coord2gridIndex(const Eigen::Vector3d &pt) {
    Eigen::Vector3i idx;
    idx << std::min(std::max(int((pt(0) - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1),
            std::min(std::max(int((pt(1) - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1),
            std::min(std::max(int((pt(2) - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

void GlobalMap::coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx)
{
    x_idx = std::min(std::max(int((pt_x - gl_xl) * m_inv_resolution), 0), GLX_SIZE - 1);
    y_idx = std::min(std::max(int((pt_y - gl_yl) * m_inv_resolution), 0), GLY_SIZE - 1);
    z_idx = std::min(std::max(int((pt_z - gl_zl) * m_inv_resolution), 0), GLZ_SIZE - 1);
}

inline void GlobalMap::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

bool GlobalMap::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

bool GlobalMap::isLocalOccupied(const Eigen::Vector3i &index) const
{
    return isLocalOccupied(index(0), index(1), index(2));
}

bool GlobalMap::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

bool GlobalMap::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 &&
            (data[idx_x * GLY_SIZE + idx_y] == 1 || l_data[idx_x * GLY_SIZE + idx_y] > 0));
}

bool GlobalMap::isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (l_data[idx_x * GLY_SIZE + idx_y] > 0));

}

bool GlobalMap::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLY_SIZE + idx_y] < 1 && l_data[idx_x * GLY_SIZE + idx_y] == 0));
}

// Fix 61a: Precompute dilated static occupancy map.
// For every static-occupied cell, mark all cells within ±NEAR_STATIC_RADIUS
// as "near static".  This turns the O(169) inner loop in linearation's
// searchWindow and the O(625) loop in checkfeasible into single array lookups,
// eliminating ~640k grid reads per MPC callback.
void GlobalMap::computeNearStaticMap() {
    near_static_data = new uint8_t[GLXY_SIZE];
    memset(near_static_data, 0, GLXY_SIZE * sizeof(uint8_t));
    int count = 0;
    for (int x = 0; x < GLX_SIZE; x++) {
        for (int y = 0; y < GLY_SIZE; y++) {
            if (data[x * GLY_SIZE + y] == 1) {
                count++;
                int x_lo = std::max(0, x - NEAR_STATIC_RADIUS);
                int x_hi = std::min(GLX_SIZE - 1, x + NEAR_STATIC_RADIUS);
                int y_lo = std::max(0, y - NEAR_STATIC_RADIUS);
                int y_hi = std::min(GLY_SIZE - 1, y + NEAR_STATIC_RADIUS);
                for (int nx = x_lo; nx <= x_hi; nx++) {
                    for (int ny = y_lo; ny <= y_hi; ny++) {
                        near_static_data[nx * GLY_SIZE + ny] = 1;
                    }
                }
            }
        }
    }
    ROS_INFO("[Fix61a] computeNearStaticMap: %d static cells dilated with radius %d", count, NEAR_STATIC_RADIUS);
}
