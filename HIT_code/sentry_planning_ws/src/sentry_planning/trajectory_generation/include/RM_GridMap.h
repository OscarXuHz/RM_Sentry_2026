//
// Created by hitcrt on 2023/5/4.
//

#ifndef SENTRY_PLANNING_RM_GRIDMAP_H
#define SENTRY_PLANNING_RM_GRIDMAP_H


#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "backward.hpp"
#include "node.h"
#include <cmath>
#include <queue>
#include <gazebo_msgs/ModelStates.h>

class GlobalMap
{
public:
    uint8_t *data;	 // data存放地图的障碍物信息（data[i]为1说明i这个栅格是障碍物栅格）
    uint8_t *l_data; // l_data存放局部地图的障碍物信息（l_data[i]为1说明i这个栅格是障碍物栅格）只用于判断地图对应高度的局部占用情况


    int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 地图尺寸
    int GLXYZ_SIZE, GLYZ_SIZE, GLXY_SIZE;
    double m_resolution, m_inv_resolution; // 栅格长度以及1m内的栅格数目

    GridNodePtr **GridNodeMap;   /// 2D地图
    GridNodePtr **GridNodeLocalMap;  /// 局部地图，两个地图用于全局规划和轨迹优化

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_local_cloud;  // 回调的局部点云地图
    Eigen::Vector3d odom_position;
    Eigen::Vector3d odom_posture;

    bool m_localmap_update;  // 局部地图是否已被规划
    std::vector<Eigen::Vector3d> topo_sample_map;
    std::vector<Eigen::Vector3d> topo_keypoint;

    std::vector<Eigen::Vector3d> second_heights_points;  // 处理桥洞高度

    void initGridMap(ros::NodeHandle &nh, std::string occ_map_file, std::string bev_map_file, std::string distance_map_file, double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                     int max_x_id, int max_y_id, int max_z_id, double robot_radius,
                     double _2d_search_height_low, double _2d_search_height_high, double _search_radius);
    /// 设置2D 1D 点云 与局部的地图
    void bevMapToHeight(const cv::Mat bev_map, const cv::Mat occ_map);
    void topoSampleMap(cv::Mat topo_map);
    void pruneGraph(cv::Mat &topo_map);
    void occMaptoObstacle(const cv::Mat occ_map);
    void processSecondHeights();
    void localPointCloudToObstacle(const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &swell_flag, const
                                   Eigen::Vector3d current_position);

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    void coord2gridIndex(double &pt_x, double &pt_y, double &pt_z, int &x_idx, int &y_idx, int &z_idx);

    double getHeight(const int idx_x, const int idx_y);
    double getSecondHeight(const int idx_x, const int idx_y);
    void setRadiusDash(const double dash_radius);
    cv::Mat swellOccMap(cv::Mat occ_map);

    void resetUsedGrids();


    bool isOccupied(const int &idx_x, const int &idx_y, const int &idx_z, bool second_height) const;
    bool isOccupied(const Eigen::Vector3i &index, bool second_height) const;
    bool isLocalOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isLocalOccupied(const Eigen::Vector3i &index) const;
    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isFree(const Eigen::Vector3i &index) const;

private:

    ros::NodeHandle m_node;
    double m_height_bias;  // (height = m_height_interval / 255.0 + m_height_bias)
    double m_height_interval;
    double m_height_threshold;
    double m_height_sencond_high_threshold;

    double m_2d_search_height_low;	// 2维规划障碍物搜索高度最低点
    double m_2d_search_height_high; // 2维规划障碍物搜索高度最高点
    double m_robot_radius;			// 机器人的碰撞检测直径 建议<0.38
    double m_robot_radius_dash;		// 机器人的动态障碍物冲卡检测直径 建议<0.25

    double gl_xl, gl_yl, gl_zl;			   // 地图的左下角坐标
    double gl_xu, gl_yu, gl_zu;			   // 地图的右上角坐标
    double search_radius;

    Eigen::Vector3i current_position_index;
//    ros::Timer m_occ_timer;
    ros::Subscriber m_local_point_sub, m_odometry_sub, gazebo_point_sub, gazebo_odometry_sub, m_realsense_sub;
    ros::Publisher global_map_pub;

    void setObs(const double coord_x, const double coord_y);
    void localSetObs(const double coord_x, const double coord_y, const double coord_z);
    void setHeight(const double coord_x, const double coord_y, const double coord_z, const bool exist_heights);

    void resetGrid(GridNodePtr ptr);

    void gazeboPosCallback(const gazebo_msgs::ModelStatesConstPtr &state);
    void gazeboCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);
    void odometryCallback(const nav_msgs::OdometryConstPtr &state);
    void lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);
    void realsenseCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point);


};

#endif //SENTRY_PLANNING_RM_GRIDMAP_H
