//
// Created by zzt on 23-10-22.
//

#ifndef TRAJECTORY_GENERATION_TOPOSEARCH_H
#define TRAJECTORY_GENERATION_TOPOSEARCH_H

#include <random>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>
#include <vector>
#include "RM_GridMap.h"

/// 定义topo路径搜索数据类型
class GraphNode
{
public:
    enum NODE_TYPE{
        Guard = 1,
        Connector = 2
    };
    enum NODE_STATE{
        NEW = 1,
        CLOSE = 2,
        OPEN = 3
    };

    GraphNode() {}
    GraphNode(Eigen::Vector3d position, NODE_TYPE type, int id) {
        pos = position;
        type_ = type;
        state_ = NEW;
        m_id = id;
    }
    ~GraphNode() {}

    std::vector<std::shared_ptr<GraphNode>> neighbors;
    std::vector<std::shared_ptr<GraphNode>> neighbors_but_noconnected;
    NODE_TYPE type_;
    NODE_STATE state_;

    Eigen::Vector3d pos;
    int m_id;
    typedef std::shared_ptr<GraphNode> Ptr;
};

class TopoSearcher{
public:
    std::shared_ptr<GlobalMap> global_map;  // 地图

    // 定义随机采样器
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;

    // 定义采样点时的变换矩阵与采样矩阵
    Eigen::Vector3d m_sample_r;
    Eigen::Vector3d m_translation;
    Eigen::Matrix3d m_rotation;
    Eigen::Matrix3d m_rotation_inv;
    Eigen::Vector3d m_sample_inflate;  /// 采样膨胀点

    std::vector<GraphNode::Ptr> m_graph;  /// 得到采样图
    std::vector<std::vector<Eigen::Vector3d>> raw_paths;
    std::vector<std::vector<Eigen::Vector3d>> final_paths;  // k 最优
    std::vector<Eigen::Vector3d> min_path;

    std::multimap<double, std::vector<Eigen::Vector3d>> distanceSet;

    double max_sample_time;
    int max_sample_num;
    double m_resolution;

    void init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &_global_map);
    Eigen::Vector3d getSample();
    Eigen::Vector3d getLocalSample();
    void createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
    void createLocalGraph(Eigen::Vector3d start, Eigen::Vector3d end, bool attack_target = false);
    bool searchOnePointPath(Eigen::Vector3d end);
    std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual);  // find pairs of visibile guard
    std::vector<GraphNode::Ptr> findMinVisibGuard(Eigen::Vector3d pt, int &exist_unvisiual);  // find Min distance pairs of visibile guard
    bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);  // test redundancy with existing
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh, Eigen::Vector3d& pc, int caster_id = 0);
    bool heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int &direction, double step = 0.05, double thresh = 0.10);
    bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
    bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1, const std::vector<Eigen::Vector3d>& path2, double thresh);
    void pruneGraph();
    std::vector<std::vector<Eigen::Vector3d>> searchPaths(int node_id = 1);

    void DijkstraSearch(int node_id);
    void depthFirstSearch(std::vector<GraphNode::Ptr>& vis);
    void checkDistanceFinalPath();

    double pathLength(const std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3d> discretizePath(std::vector<Eigen::Vector3d> path, int pt_num);

    int checkSearchDirection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);
    void setSearchDirection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id, int direction);

    void checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id);
};


#endif //TRAJECTORY_GENERATION_TOPOSEARCH_H
