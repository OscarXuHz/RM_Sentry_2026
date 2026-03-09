#ifndef RM_PLANNING_MANAGER_H
#define RM_PLANNING_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int64.h>

#include <cmath>
#include <queue>
#include <algorithm>

#include "Astar_searcher.h"
#include "path_smooth.h"
#include "reference_path.h"
#include "TopoSearch.h"

class planner_manager
{
public:

    /** A*, 全局地图， 路径平滑和参考轨迹相关对象 **/
    std::unique_ptr<AstarPathFinder> astar_path_finder;
    std::shared_ptr<GlobalMap> global_map;
    std::unique_ptr<Smoother> path_smoother;
    std::unique_ptr<Refenecesmooth> reference_path;
    std::unique_ptr<TopoSearcher> topo_prm;

    // 定义随机采样器
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;

    /// 参考路径的最大线avw
    double reference_v_max;
    double reference_a_max;
    double reference_w_max;
    double reference_desire_speed;
    double reference_desire_speedxtl;

    std::vector<std::vector<Eigen::Vector3d>> sample_path_set;

    /// 小陀螺或者小虎步时的avw
    double reference_vxtl_max;
    double reference_axtl_max;
    double reference_wxtl_max;

    /* 相关标志位 */
    bool isxtl;    // 电控陀螺标志位
    bool xtl_flag; // 规划陀螺标志位

    /** 地图相关参数：四个初始地图及地图参数 **/
    std::string map_file_path;
    std::string occ_file_path;
    std::string bev_file_path;
    std::string distance_map_file_path;

    double map_resolution;
    double map_inv_resolution;
    double map_x_size;
    double map_y_size;
    double map_z_size;
    Eigen::Vector3d map_lower_point;  /// 地图的左上角和右下角
    Eigen::Vector3d map_upper_point;
    double search_height_min;         /*局部点云可视范围*/
    double search_height_max;
    double search_radius;

    /* 路径查找相关起点终点和仿真特有的地图偏执*/
    Eigen::Vector3d start_point;
    Eigen::Vector3d target_point;
    Eigen::Vector3d map_offset;

    int grid_max_id_x;
    int grid_max_id_y;
    int grid_max_id_z;
    int raycast_num;
    int global_planning_times;
    bool current_pos_init_flag;
    bool visualization_flag;

    /* 放在replan中机器人相关位置，速度，yaw角速度，姿态状态 */
    Eigen::Vector3d robot_cur_position;
    Eigen::Quaterniond robot_cur_orientation;
    Eigen::Vector2d robot_cur_speed;
    double robot_line_speed = 0.0;
    double robot_angular = 0.0;
    double robot_cur_yaw = 0.0;

    /// 机器人点云膨胀半径，太大了不好哦
    double robot_radius;
    double robot_radius_dash;

    /* 相关标志位 */
    bool obstacle_swell_flag;
    bool obstacle_swell_vis_flag;

    /*全局规划结果与局部规划结果*/
    std::vector<Eigen::Vector3d> optimized_path;
    std::vector<Eigen::Vector3d> local_optimize_path;
    std::vector<Eigen::Vector3d> ref_trajectory;  // 可视化和重规划判断
    std::vector<Eigen::Vector3d> astar_path;
    std::vector<Eigen::Vector2d> final_path;
    std::vector<Eigen::Vector2d> final_path_temp;  // 临时可视化变量
    std::vector<GraphNode::Ptr> global_graph;

    /*决策相关变量*/
    int decision_mode;
    int enemy_num;           /*敌人数量*/
    int sentry_HP;           /*烧饼血量*/
    bool is_attacked;        /*是否被攻击*/
    int countHP;             /*血量统计*/

    double last_current_yaw;
    typedef enum {
        red = 0,
        blue,
    } teamColor;
    teamColor sentryColor;

    // 全局规划结果

    /* 路径查找相关函数 */
    void init(ros::NodeHandle &nh);
    std::vector<Eigen::Vector3d> localPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
    bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                     const Eigen::Vector3d start_vel);
    bool replanFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,
                       const Eigen::Vector3d start_vel);
    void pubGlobalPlanningResult(std::vector<Eigen::Vector3d> nodes);
//    bool AstarGlobalResult(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);

};

#endif