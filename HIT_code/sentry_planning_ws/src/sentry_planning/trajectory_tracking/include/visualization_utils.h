//
// Created by hitcrt on 2023/5/6.
//

#ifndef SENTRY_PLANNING_VISUALIZATION_UTILS_H
#define SENTRY_PLANNING_VISUALIZATION_UTILS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>


class Vislization
{
public:
    ros::Publisher candidate_path_vis_pub, reference_path_vis_pub, obs_center_vis_pub;

    void init(ros::NodeHandle &nh);

    void visCandidateTrajectory(std::vector<Eigen::Vector4d> points);

    void visReferenceTrajectory(std::vector<Eigen::Vector3d> points);

    void visObsCenterPoints(std::vector<std::vector<Eigen::Vector3d>> points);

};

#endif //SENTRY_PLANNING_VISUALIZATION_UTILS_H
