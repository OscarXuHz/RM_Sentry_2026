#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>

class DBSCAN {
public:
    DBSCAN(double radius, int minPts) : radius_(radius), minPts_(minPts) {}

    void run(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        if (cloud->points.empty()) return;

        kdTree_.setInputCloud(cloud);
        points_.resize(cloud->points.size());

        // 初始化点状态为未分类
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            points_[i].point = cloud->points[i];
            points_[i].cluster = -1;  // -1: unclassified
        }

        int clusterId = 0;
        for (size_t i = 0; i < points_.size(); ++i) {
            if (points_[i].cluster == -1) {  // 只处理未分类点
                std::vector<int> neighbors = getNeighbors(points_[i].point);
                if (neighbors.size() >= minPts_) {
                    expandCluster(i, neighbors, clusterId);
                } else {
                    points_[i].cluster = -2;  // 标记为噪声
                }
            }
        }

        filterNoise(cloud);
    }

private:
    struct PointXYZICluster {
        pcl::PointXYZI point;
        int cluster;  // -1: unclassified, -2: noise, >=0: cluster ID
    };

    double radius_;
    int minPts_;
    pcl::search::KdTree<pcl::PointXYZI> kdTree_;
    std::vector<PointXYZICluster> points_;

    std::vector<int> getNeighbors(const pcl::PointXYZI& point) {
        std::vector<int> indices;
        std::vector<float> distances;
        kdTree_.radiusSearch(point, radius_, indices, distances, minPts_);  // 限制返回数量
        return indices;
    }

    void expandCluster(int pointIdx, const std::vector<int>& neighbors, int clusterId) {
        points_[pointIdx].cluster = clusterId;
        std::queue<int> neighborQueue;
        for (int idx : neighbors) {
            if (points_[idx].cluster == -1) {  // 只处理未分类点
                neighborQueue.push(idx);
            }
        }

        while (!neighborQueue.empty()) {
            int idx = neighborQueue.front();
            neighborQueue.pop();

            if (points_[idx].cluster == -1 || points_[idx].cluster == -2) {
                points_[idx].cluster = clusterId;
                std::vector<int> newNeighbors = getNeighbors(points_[idx].point);
                for (int newIdx : newNeighbors) {
                        if (points_[newIdx].cluster == -1) {
                            neighborQueue.push(newIdx);
                        }
                    }
                // if (newNeighbors.size() >= minPts_) {
                   
                // }
            }
        }
    }

    void filterNoise(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& pointCluster : points_) {
            if (pointCluster.cluster != -2) {  // 排除噪声点
                filteredCloud->points.push_back(pointCluster.point);
            }
        }
        *cloud = *filteredCloud;
    }
};
