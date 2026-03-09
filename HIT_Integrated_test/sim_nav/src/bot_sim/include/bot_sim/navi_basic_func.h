#pragma once

#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
// #include <ceres/ceres.h> //失败的尝试
// #include <glog/logging.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <tf/transform_datatypes.h>
#include <deque>

extern int max_radar_x;
extern int max_radar_y;
extern int min_radar_x;
extern int min_radar_y;
extern int MAXN;


//距离平方计算函数 需要多次调用 sqrt开销大 故使用距离平方
int calculate_distance_sq(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return static_cast<int>(dx * dx + dy * dy);
}

// 计算网格的权值 模糊化处理
int calculate_point_value(const std::vector<std::vector<int>>& grid, int x, int y, int height, int width)
{
    int sum = 0;

    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            int nx = x + i;
            int ny = y + j;

            if (nx >= 0 && nx < height && ny >= 0 && ny < width) {
                // 计算权值
                double weight = (i == 0 || j == 0) ? 0.15 : 0.1;
                sum += weight * grid[nx][ny];
            }
        }
    }

    return sum;
}

inline int my_abs(int x){
    return x>0 ? x : -x;
}

inline int my_max(int x, int y){
    return x>y ? x : y;
}

inline int my_min(int x, int y){
    return x<y ? x : y;
}
//反距离插值函数
int inverse_distance_weighting(const std::vector<std::vector<int>>& data, int x, int y, int r) {
    float sum_values = 0.0;
    int distance=0;
    if (data[x][y]>=80) return 100;//已知点无需计算
    if (x+r<min_radar_x || x-r>max_radar_x || y+r<min_radar_y || y-r>max_radar_y) return 0; //优化周围无雷达数据的点
    int li=my_max(-r,-x),ri=my_min(MAXN-x-1,r),lj=my_max(-r,-y),rj=my_min(MAXN-y-1,r);//逃过超出边界的点
    for (int i = li; i<=ri ; i++){
        for (int j = lj; j<=rj ; j++){
            // if (x+i < 0 || x+i >= MAXN || y+j < 0 || y+j >= MAXN) continue;//逃过超出边界的点
            if (data[x+i][y+j] == 0) continue; //优化无贡献点
            distance = my_abs(i) + my_abs(j) + 1;
            sum_values += 1.0 / (std::sqrt(distance * distance * distance)) * data[x+i][y+j];
        }
    }
    int result = sum_values;
    return (result>100 ? 100 :result);
}


// calculate slope
inline float tangent(float x1, float y1, float x2, float y2){
    float dx = x2 - x1;
    float dy = y2 - y1;
    if ((dx == 0)|| (dy > 0)) return 5000;
    if ((dx == 0)|| (dy < 0)) return -5000;
    return dy/dx;
}

inline float precise_distance(float x1, float y1, float x2, float y2){
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// 偏差计算
inline int monotonicity_judge(float a, float b, float c){
    if ((a<=b && b<=c) || (a>=b && b>=c))return 1;
    return -1;
}

//定义修正函数
inline float calc_deviation(float d1, float d2, float d3){
    //theta_square = 0.000279328; //theta = 0.0167131
    return sqrt(std::abs((2*d2-d1-d3)*(d2-d1)));
}

void filling_func(const std::vector<std::vector<float>>& laser_init_data,std::vector<std::vector<int>>& data, int data_size){
    float epsilon_theta=0.7,threshold=5.0, deviation=0.0,epsilon_d=3.0, memoslop=0.0, slop=0.0, x_0=0, y_0=0, d_1=0, d_2=0; 
    int count=0,grid_x=0, grid_y=0, dx=0, dy=0, sign=0;
    float x_1 = laser_init_data[0][0];
    float y_1 = laser_init_data[0][1];
    float x_2 = laser_init_data[1][0];
    float y_2 = laser_init_data[1][1];
    float d_o_1 = laser_init_data[0][2]; // distance to origin
    float d_o_2 = laser_init_data[1][2];
    float d_o_3 = 0;
    memoslop = tangent(x_1,y_1,x_2,y_2);
    d_1=precise_distance(x_1,y_1,x_2,y_2); // distance between two radar points
    for (int i = 2; i<data_size; i++){
        x_1 = x_2; 
        y_1 = y_2;
        x_2 = laser_init_data[i][0];
        y_2 = laser_init_data[i][1];
        d_o_3 = laser_init_data[i][2];
        d_2=precise_distance(x_1,y_1,x_2,y_2);
        slop = tangent(x_1,y_1,x_2,y_2);
        sign = monotonicity_judge(d_o_1,d_o_2,d_o_3);
        //剔除无效点
        if (x_1==-10000 || y_1==-10000 || x_2==-10000 || y_2==-10000) { 
            d_1=d_2; d_o_1=d_o_2; d_o_2=d_o_3; memoslop = slop;
            continue;
        }
        //处理相邻间断点
        if ((d_1<=7 && d_2 <=7) && (((memoslop>=100|| memoslop<=-100) && slop<=5 && slop>=-5) || ((slop>=100|| slop<=-100) && memoslop<=5 && memoslop>=-5))){
            x_0 = x_2-x_1;
            y_0 = y_2-y_1;
            int increment = std::max(std::abs(x_0),std::abs(y_0));
            float dy = y_0/increment;
            float dx = x_0/increment;
            for (int j = 1; j<= increment; j++){
                grid_x = x_1 + j*dx;
                grid_y = y_1 + j*dy;
                data[grid_y][grid_x] = 100;
            }
        } 
        // 正常处理
        else if (((std::abs(slop-memoslop) < epsilon_theta) && (std::abs(d_1-sign*d_2) < epsilon_d + calc_deviation(d_o_1,d_o_2,d_o_3))) || (d_1<=5 && d_2<=5)){
            x_0 = x_2-x_1;
            y_0 = y_2-y_1;
            int increment = std::max(std::abs(x_0),std::abs(y_0));
            float dy = y_0/increment;
            float dx = x_0/increment;
            for (int j = 1; j<= increment; j++){
                grid_x = x_1 + j*dx;
                grid_y = y_1 + j*dy;
                data[grid_y][grid_x] = 100;
            }
        }
        d_1=d_2;
        memoslop = slop;
        d_o_1=d_o_2; d_o_2=d_o_3;
    }
}

