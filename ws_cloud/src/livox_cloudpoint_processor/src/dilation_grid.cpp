#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>


void dilateOccupancyGrid(nav_msgs::OccupancyGrid& grid, double robot_radius) {
    // 检查栅格地图分辨率是否有效
    if (grid.info.resolution <= 0 || robot_radius <= 0) {
        ROS_WARN("Invalid resolution or robot_radius");
        return;
    }

    // 获取地图的宽度、高度和分辨率
    const int width = grid.info.width;
    const int height = grid.info.height;
    const double resolution = grid.info.resolution;

    // 计算膨胀范围（以栅格数为单位）
    const int inflation_radius = static_cast<int>(std::ceil(robot_radius / resolution));

    // 原始地图数据
    const std::vector<int8_t>& original_data = grid.data;
    std::vector<int8_t> inflated_data = original_data;

    // 创建一个队列，用于存储障碍物点
    std::queue<std::pair<int, int>> obstacle_queue;

    // 遍历地图，寻找障碍物点
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (original_data[index] > 0) { // 障碍物点（占据值大于0）
                obstacle_queue.push({x, y});
            }
        }
    }

    // 用于计算膨胀代价的缓存
    std::vector<std::vector<double>> cached_distances(inflation_radius + 1, std::vector<double>(inflation_radius + 1, 0));
    for (int dx = 0; dx <= inflation_radius; ++dx) {
        for (int dy = 0; dy <= inflation_radius; ++dy) {
            cached_distances[dx][dy] = std::hypot(dx, dy);
        }
    }

    // 执行膨胀操作
    while (!obstacle_queue.empty()) {
        std::pair<int, int> obstacle = obstacle_queue.front();
        int ox = obstacle.first;
        int oy = obstacle.second;
        obstacle_queue.pop();

        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
            for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                int nx = ox + dx;
                int ny = oy + dy;

                // 检查新点是否在地图范围内
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }

                int new_index = ny * width + nx;

                // if (inflated_data[new_index] == -1) { // -1 表示未知区域
                //     continue;
                // }

                double distance = cached_distances[std::abs(dx)][std::abs(dy)];

                // 如果距离超过膨胀半径，则跳过
                if (distance > inflation_radius) {
                    continue;
                }
                // 线性衰减计算代价：距离越近代价越大，远离障碍物代价逐渐减小
                int cost = static_cast<int>(100 * (1.0 - (distance / inflation_radius)));  // [100, 1]
                cost = std::max(1, cost); // 确保代价至少为1
                // 如果当前点的代价已经小于新的代价，则继续膨胀
                if (inflated_data[new_index] < cost) {
                    inflated_data[new_index] = cost;
                    // 输出膨胀点
                    // ROS_INFO("Inflated point: (%d, %d), cost: %d", nx, ny, cost);
                }
            }
        }
    }

    // 更新栅格地图数据
    grid.data = inflated_data;
}
