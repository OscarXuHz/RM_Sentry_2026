//
// Created by hitcrt on 2023/5/6.
//

#include "../include/visualization_utils.h"


/**
 * @brief   可视化当前位置
 *
 * @param
 */
void Vislization::init(ros::NodeHandle &nh)
{
    candidate_path_vis_pub = nh.advertise<visualization_msgs::Marker>("candidate_path_vis", 1);
    reference_path_vis_pub = nh.advertise<visualization_msgs::Marker>("reference_path_vis", 1);
    obs_center_vis_pub = nh.advertise<visualization_msgs::Marker>("obs_center_vis", 1);
}

void Vislization::visCandidateTrajectory(std::vector<Eigen::Vector4d> nodes) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "candidate_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.5;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        pt.x = nodes[i](0);
        pt.y = nodes[i](1);
        pt.z = 0.0;

        node_vis.points.push_back(pt);
    }

    candidate_path_vis_pub.publish(node_vis);
}

void Vislization::visReferenceTrajectory(std::vector<Eigen::Vector3d> nodes) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "candidate_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.8;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        pt.x = nodes[i](0);
        pt.y = nodes[i](1);
        pt.z = 0.0;

        node_vis.points.push_back(pt);
    }

    reference_path_vis_pub.publish(node_vis);
}

void Vislization::visObsCenterPoints(std::vector<std::vector<Eigen::Vector3d>> points)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "obs_points";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.8;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(points.size()); i++) {
        for(int j = 0; j<int(points[i].size()); j++)
        {
            pt.x = points[i][j](0);
            pt.y = points[i][j](1);
            pt.z = 0.0;

            node_vis.points.push_back(pt);
        }
    }

    obs_center_vis_pub.publish(node_vis);
}