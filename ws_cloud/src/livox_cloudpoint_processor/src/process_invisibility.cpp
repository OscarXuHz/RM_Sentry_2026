#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <ros/console.h>

// 光线模拟函数，模拟从中心到目标点的路径，标记不可见区域
void markInvisibleArea(nav_msgs::OccupancyGrid& grid, int tx, int ty) {
    const int w = grid.info.width;
    const int h = grid.info.height;
    const int cx = w / 2;
    const int cy = h / 2;

    // if (tx < 0 || tx >= w || ty < 0 || ty >= h) {
    //     return;
    // }

    int dx = std::abs(tx - cx);
    int dy = std::abs(ty - cy);

    int outer_steps = std::min(dx, dy);
    bool use_x_as_outer = dx < dy;

    int inner_steps, remainder;
    // 根据x和y的大小确定内外循环次数
    if (dx == 0 || dy == 0) {
        // 如果 dx 或 dy 为零，只进行内循环
        outer_steps = 0;
        inner_steps = (dx == 0) ? dy : dx;
        remainder = 0;
    } else {
        if (use_x_as_outer) {
            inner_steps = dy / dx;
            remainder = dy % dx;
        } else {
            inner_steps = dx / dy;
            remainder = dx % dy;
        }
    }

    bool blocked = false;

    int nx = cx;
    int ny = cy;
    for (int i = 0; i <= outer_steps; ++i) {
        // 更新 nx 和 ny
        nx = nx + (use_x_as_outer ? (tx == cx ? 0 : (tx > cx ? 1 : -1)) : 0);
        ny = ny + (use_x_as_outer ? 0 : (ty == cy ? 0 : (ty > cy ? 1 : -1)));

        // 根据余数调整内循环次数
        int step_count = (i < remainder) ? inner_steps + 1 : inner_steps;

        for (int j = 0; j < step_count; ++j) {
            int idx = ny * w + nx;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
                continue;
            }
            // 更新 nx 和 ny
            nx = nx + (use_x_as_outer ?  0 : (tx == cx ? 0 : (tx > cx ? 1 : -1)));
            ny = ny + (use_x_as_outer ? (ty == cy ? 0 : (ty > cy ? 1 : -1)) : 0);
            // 确保索引有效
            
            // 判断是否遇到障碍物
            if (grid.data[idx] ==100 && blocked == false) {
                blocked = true;
                continue;
            }
            // 如果之前遇到障碍物，标记后续区域为不可见
            if (blocked && grid.data[idx] != 100) {
                grid.data[idx] = -1;
            }
        }
    }
}


// BFS 遍历可视区域
void bfsVisibility(nav_msgs::OccupancyGrid& grid, int start_x, int start_y, std::vector<int8_t>& visible_map) {
    const int width = grid.info.width;
    const int height = grid.info.height;
    
    // BFS 队列
    std::queue<std::pair<int, int>> q;
    q.push({start_x, start_y});
    visible_map[start_y * width + start_x] = 1; // 标记起始点为可见

    // 定义 4 个方向
    int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!q.empty()) {
        int x = q.front().first;
        int y = q.front().second;
        q.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = x + directions[i][0];
            int ny = y + directions[i][1];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height && visible_map[ny * width + nx] != 1) {
            // 判断是否为可见区域
                if (grid.data[ny * width + nx] != -1 && grid.data[ny * width + nx] != 100) {
                    visible_map[ny * width + nx] = 1;
                    q.push({nx, ny});
                }
            }
        }
    }
}

// 更新膨胀地图，将不可见区域标记为 -1
void updateInflatedMap(nav_msgs::OccupancyGrid& inflated_grid, const std::vector<int8_t>& visible_map) {
    const int width = inflated_grid.info.width;
    const int height = inflated_grid.info.height;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            // 如果原地图值为 100 或可视地图值为 -1，则标记为不可见
            if (inflated_grid.data[index] != 100 && visible_map[index] != 1) {
                inflated_grid.data[index] = -1;
            }
        }
    }
}

void processVisibility(nav_msgs::OccupancyGrid& grid) {
    const int width = grid.info.width*2;
    const int height = grid.info.height*2;
    // ROS_INFO("Processing visibility for map %dx%d", width, height);
    std::vector<int8_t> visible_map(width * height, 0); // 初始所有区域不可见

    // 模拟光路照向边界上所有的点
    for (int x = 0; x < width; ++x) {
        // 上边界
        markInvisibleArea(grid, x, 0);
        // 下边界
        markInvisibleArea(grid, x, height - 1);
    }
    for (int y = 0; y < height; ++y) {
        // 左边界
        markInvisibleArea(grid, 0, y);
        // 右边界
        markInvisibleArea(grid, width - 1, y);
    }

    // 使用 BFS 获取可视区域
    // bfsVisibility(grid, width / 2, height / 2, visible_map);

    // 更新膨胀地图
    // updateInflatedMap(grid, visible_map);
}