//
// Created by hitcrt on 2023/5/5.
//

#ifndef SENTRY_PLANNING_VISUALIZATION_UTILS_H
#define SENTRY_PLANNING_VISUALIZATION_UTILS_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "Astar_searcher.h"
#include "TopoSearch.h"

class Vislization
{
public:
    ros::Publisher astar_path_vis_pub, optimized_path_vis_pub, cur_position_vis_pub, obs_vis_pub;
    ros::Publisher reference_path_vis_pub, final_path_vis_pub, target_position_vis_pub, final_line_strip_pub;
    ros::Publisher topo_position_guard_vis_pub, topo_position_connection_vis_pub, topo_line_vis_pub;
    ros::Publisher topo_path_point_vis_pub, topo_path_vis_pub, attack_target_vis_pub;

    void init(ros::NodeHandle &nh);

    void visAstarPath(std::vector <Eigen::Vector3d> nodes);  // Astar原始路径

    void visFinalPath(std::vector <Eigen::Vector3d> nodes);  // Astar关键点

    void visOptimizedPath(std::vector<Eigen::Vector2d> nodes);  // 路径点优化后的路径

    void visOptGlobalPath(const std::vector <Eigen::Vector3d> &nodes);  // 最终的优化轨迹

    void visCurPosition(const Eigen::Vector3d cur_pt);  // 机器人当前的位置

    void visTargetPosition(const Eigen::Vector3d cur_pt);

    void visObs(std::vector<std::vector<Eigen::Vector3d>> nodes);

    void visTopoPointGuard(std::vector<GraphNode::Ptr> global_graph);

    void visTopoPointConnection(std::vector<GraphNode::Ptr> global_graph);

    void visTopoPath(std::vector<std::vector<Eigen::Vector3d>> path);

};




#endif //SENTRY_PLANNING_VISUALIZATION_UTILS_H
