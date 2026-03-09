//
// Created by hitcrt on 2023/5/5.
//

#include "../include/visualization_utils.h"


void Vislization::init(ros::NodeHandle &nh)
{
    astar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("astar_path_vis", 1);
    final_path_vis_pub = nh.advertise<visualization_msgs::Marker>("final_path_vis_pub", 1);
    final_line_strip_pub = nh.advertise<visualization_msgs::Marker>("final_line_strip_pub", 1);
    optimized_path_vis_pub = nh.advertise<visualization_msgs::Marker>("optimized_path_vis", 1);
    cur_position_vis_pub = nh.advertise<visualization_msgs::Marker>("cur_position_vis", 1);
    target_position_vis_pub = nh.advertise<visualization_msgs::Marker>("target", 1);
    topo_position_guard_vis_pub = nh.advertise<visualization_msgs::Marker>("topo_point_guard", 1);
    topo_position_connection_vis_pub = nh.advertise<visualization_msgs::Marker>("topo_point_connection", 1);
    topo_line_vis_pub = nh.advertise<visualization_msgs::Marker>("topo_line", 1);
    reference_path_vis_pub = nh.advertise<visualization_msgs::Marker>("reference_path", 1);
    obs_vis_pub = nh.advertise<visualization_msgs::Marker>("obs", 1);
    topo_path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("topo_point", 1);
    topo_path_vis_pub = nh.advertise<visualization_msgs::Marker>("topo_point_path", 1);
}


/**
 * @brief	将规划出来的栅格路径发布到RVIZ
 *
 * @param
 *
 */
void Vislization::visAstarPath(std::vector <Eigen::Vector3d> nodes) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 0.4;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    astar_path_vis_pub.publish(node_vis);
}

void Vislization::visObs(std::vector<std::vector<Eigen::Vector3d>> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "obs";
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
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 1.0;

    geometry_msgs::Point pt;
    if(!nodes.empty()) {
        for (int j = 0; j < 1; j++) {
            for (int i = 0; i < int(nodes[j].size()); i++) {
                Eigen::Vector3d coord = nodes[nodes.size() - 3][i];
                pt.x = coord(0);
                pt.y = coord(1);
                pt.z = 0.0;

                node_vis.points.push_back(pt);
            }
        }
    }

    obs_vis_pub.publish(node_vis);
}

void Vislization::visFinalPath(std::vector <Eigen::Vector3d> nodes) {
    visualization_msgs::Marker node_vis, line_strip;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "optimized_path";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.8;
    node_vis.color.r = 0.7;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.4;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 1.0;

    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "optimized_path";
    line_strip.type = visualization_msgs::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 1;

    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;

    line_strip.scale.x = 0.04;
    line_strip.scale.y = 0.04;
    line_strip.scale.z = 0.04;

    geometry_msgs::Point pt, neigh_pt;
    for (int i = 0; i < int(nodes.size()) - 1; i++) {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        neigh_pt.x = nodes[i+1](0);
        neigh_pt.y = nodes[i+1](1);
        neigh_pt.z = nodes[i+1](2);

        line_strip.points.push_back(pt);
        line_strip.points.push_back(neigh_pt);

        node_vis.points.push_back(pt);
    }

    final_path_vis_pub.publish(node_vis);
    final_line_strip_pub.publish(line_strip);
}

/**
 * @brief	用lbfgs优化后的控制点（相当于降采样）后发给rviz以显示出来
 *          （二维规划下只取指定高度范围的点云）
 * @param
 */
void Vislization::visOptimizedPath(std::vector<Eigen::Vector2d> nodes) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "optimized_path";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.6;
    node_vis.color.b = 0.6;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Eigen::Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0.0;

        node_vis.points.push_back(pt);
    }

    optimized_path_vis_pub.publish(node_vis);
}

void Vislization::visOptGlobalPath(const std::vector <Eigen::Vector3d> &nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "globaltrajectory";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.04;
    node_vis.scale.y = 0.04;
    node_vis.scale.z = 0.04;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = 0.0;

        node_vis.points.push_back(pt);
    }

    reference_path_vis_pub.publish(node_vis);
}

void Vislization::visCurPosition(const Eigen::Vector3d cur_pt) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "cur_position";
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
    node_vis.color.b = 1.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 2;

    geometry_msgs::Point pt;
    Eigen::Vector3d coord = cur_pt;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = coord(2);
    node_vis.points.push_back(pt);
    cur_position_vis_pub.publish(node_vis);
}

void Vislization::visTargetPosition(const Eigen::Vector3d target_pt) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "target_position";
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
    node_vis.scale.z = 2;

    geometry_msgs::Point pt;
    Eigen::Vector3d coord = target_pt;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = coord(2);
    node_vis.points.push_back(pt);
    target_position_vis_pub.publish(node_vis);
}

void Vislization::visTopoPath(std::vector<std::vector<Eigen::Vector3d>> path)
{
    visualization_msgs::Marker node_vis, line_strip;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "topopath";
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

    node_vis.scale.x = 0.05;
    node_vis.scale.y = 0.05;
    node_vis.scale.z = 0.5;

    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "topopath";
    line_strip.type = visualization_msgs::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 1;

    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;

    line_strip.scale.x = 0.05;
    line_strip.scale.y = 0.05;
    line_strip.scale.z = 0.05;
    for(int i = 0; i < path.size(); i++) {
        for (int j = 0; j < path[i].size() - 1; j++) {

            geometry_msgs::Point pt, neigh_pt;
            pt.x = path[i][j](0);
            pt.y = path[i][j](1);
            pt.z = path[i][j](2) * 2;
            node_vis.points.push_back(pt);

            neigh_pt.x = path[i][j+1](0);
            neigh_pt.y = path[i][j+1](1);
            neigh_pt.z = path[i][j+1](2);

            line_strip.points.push_back(pt);
            line_strip.points.push_back(neigh_pt);

        }
    }
    topo_path_point_vis_pub.publish(node_vis);
    topo_path_vis_pub.publish(line_strip);
}


void Vislization::visTopoPointGuard(std::vector<GraphNode::Ptr> global_graph)
{
    visualization_msgs::Marker node_vis, line_strip;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "topopoint_guard";
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
    node_vis.color.b = 0.0;

    node_vis.scale.x = 0.05;
    node_vis.scale.y = 0.05;
    node_vis.scale.z = 0.5;

    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "topopoint_guard";
    line_strip.type = visualization_msgs::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 1;

    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;

    line_strip.scale.x = 0.02;
    line_strip.scale.z = 0.02;
    line_strip.scale.y = 0.02;



    for(std::vector<GraphNode::Ptr>::iterator iter = global_graph.begin(); iter != global_graph.end(); ++iter) {

        if((*iter)->type_ == GraphNode::Connector){
            continue;
        }
        geometry_msgs::Point pt;
        pt.x = (*iter)->pos(0);
        pt.y = (*iter)->pos(1);
        pt.z = (*iter)->pos(2) * 2;
        node_vis.points.push_back(pt);

        for(int j = 0; j < (*iter)->neighbors.size();j++){
            geometry_msgs::Point neigh_pt;
            neigh_pt.x = (*iter)->neighbors[j]->pos(0);
            neigh_pt.y = (*iter)->neighbors[j]->pos(1);
            neigh_pt.z = (*iter)->neighbors[j]->pos(2) * 2;
            line_strip.points.push_back(pt);
            line_strip.points.push_back(neigh_pt);
        }

        for(int j = 0; j < (*iter)->neighbors_but_noconnected.size();j++){
            geometry_msgs::Point neigh_pt;
            neigh_pt.x = (*iter)->neighbors_but_noconnected[j]->pos(0);
            neigh_pt.y = (*iter)->neighbors_but_noconnected[j]->pos(1);
            neigh_pt.z = (*iter)->neighbors_but_noconnected[j]->pos(2);
        }
    }
    topo_position_guard_vis_pub.publish(node_vis);
//    topo_line_vis_pub.publish(line_strip);
//    topo_path_vis_pub.publish(line_strip2);
}

void Vislization::visTopoPointConnection(std::vector<GraphNode::Ptr> global_graph)
{
    visualization_msgs::Marker node_vis,line_strip, line_strip2;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "topopoint_connect";
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

    node_vis.scale.x = 0.03;
    node_vis.scale.y = 0.03;
    node_vis.scale.z = 0.5;

    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "topopoint_guard";
    line_strip.type = visualization_msgs::Marker::LINE_LIST;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 1;

    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;

    line_strip.scale.x = 0.02;
    line_strip.scale.z = 0.02;
    line_strip.scale.y = 0.02;

    line_strip2.header.frame_id = "map";
    line_strip2.header.stamp = ros::Time::now();
    line_strip2.ns = "topopath";
    line_strip2.type = visualization_msgs::Marker::LINE_LIST;
    line_strip2.action = visualization_msgs::Marker::ADD;
    line_strip2.id = 1;

    line_strip2.pose.orientation.x = 0.0;
    line_strip2.pose.orientation.y = 0.0;
    line_strip2.pose.orientation.z = 0.0;
    line_strip2.pose.orientation.w = 1.0;
    line_strip2.color.a = 1.0;
    line_strip2.color.r = 1.0;
    line_strip2.color.g = 1.0;
    line_strip2.color.b = 0.0;

    line_strip2.scale.x = 0.04;
    line_strip2.scale.y = 0.04;
    line_strip2.scale.z = 0.04;

    for(std::vector<GraphNode::Ptr>::iterator iter = global_graph.begin(); iter != global_graph.end(); ++iter) {

        if((*iter)->type_ == GraphNode::Guard){
            continue;
        }
        geometry_msgs::Point pt;
        pt.x = (*iter)->pos(0);
        pt.y = (*iter)->pos(1);
        pt.z = (*iter)->pos(2) * 2;


        for(int j = 0; j < (*iter)->neighbors.size();j++){
            if((*iter)->neighbors.size() < 2){
                continue;
            }
            geometry_msgs::Point neigh_pt;
            neigh_pt.x = (*iter)->neighbors[j]->pos(0);
            neigh_pt.y = (*iter)->neighbors[j]->pos(1);
            neigh_pt.z = (*iter)->neighbors[j]->pos(2) * 2;
            line_strip.points.push_back(pt);
            line_strip.points.push_back(neigh_pt);
        }

        for(int j = 0; j < (*iter)->neighbors.size();j++){
            if((*iter)->neighbors.size() > 1){
                continue;
            }
            geometry_msgs::Point neigh_pt;
            neigh_pt.x = (*iter)->neighbors[j]->pos(0);
            neigh_pt.y = (*iter)->neighbors[j]->pos(1);
            neigh_pt.z = (*iter)->neighbors[j]->pos(2) * 2;
            line_strip2.points.push_back(pt);
            line_strip2.points.push_back(neigh_pt);
        }


        node_vis.points.push_back(pt);
    }
    topo_position_connection_vis_pub.publish(node_vis);
    topo_line_vis_pub.publish(line_strip);
//    topo_path_vis_pub.publish(line_strip2);
}