#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <string>
/*  lyf map
*
*      y
*      ↑
*      |
*   15 |——————————————————————————————————————————|
*      |                                          |
*      |                                          |
*      |                                          |
*   7.5|-——O1——————————————O2                     |
*      |   |               |                      |
*      |   |               |                      |
*      |   |               |                      |
*      |———|———————————————|——————————————————————|-→ x 
* (0,0)    1.57           14.00                 28 
*            
* O1: 基地  O2: 符
*/


int main(int argc, char** argv) {
    ros::init(argc, argv, "get_param_example");
    ros::NodeHandle nh;
    std::string node_name = "calculate_shift_theta";
    double O1_x, O1_y, O2_x, O2_y;

    // 尝试获取参数
    if (nh.getParam("/" + node_name + "/O1_x", O1_x)) {
        ROS_INFO("Got O1_x: %f", O1_x);
    } else {
        ROS_WARN("Failed to get param 'O1_x'");
    }
    if (nh.getParam("/" + node_name + "/O1_y", O1_y)) {
        ROS_INFO("Got O1_y: %f", O1_y);
    } else {
        ROS_WARN("Failed to get param 'O1_y'");
    }
    if (nh.getParam("/" + node_name + "/O2_x", O2_x)) {
        ROS_INFO("Got O2_x: %f", O2_x);
    } else {
        ROS_WARN("Failed to get param 'O2_x'");
    }
    if (nh.getParam("/" + node_name + "/O2_y", O2_y)) {
        ROS_INFO("Got O2_y: %f", O2_y);
    } else {
        ROS_WARN("Failed to get param 'O2_y'");
    }
    double theta = atan2(O2_y - O1_y, O2_x - O1_x);
    ROS_INFO("theta: %f", theta);
    ROS_INFO("shift_x: %f", O1_x-(1.57*cos(theta)-7.5*sin(theta)));
    ROS_INFO("shift_y: %f", O1_y-(1.57*sin(theta)+7.5*cos(theta)));

    ros::spin();
    return 0;
}