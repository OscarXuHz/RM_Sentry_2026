#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "livox_ros_driver2/CustomMsg.h"
// Include for testing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>

double first_RADIUS;
double second_RADIUS, max_height, start_height;

double slope_1,slp_first_RADIUS, height_1;
double slope_2, slp_second_RADIUS, height_2;
double slope_3,slp_third_RADIUS, height_3;

bool get_msg = 0;
std::string base_frame;
std::string laser_frame;
std::string scan_topic_left;
std::string scan_topic_right;
std::string new_scan_topic;
ros::Publisher pub;
livox_ros_driver2::CustomMsg scan_copy_left;
livox_ros_driver2::CustomMsg scan_copy_right;
geometry_msgs::TransformStamped transformStamped;
void scanCallback_left(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_left = scan;
    get_msg = 1;
}
void scanCallback_right(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_right = scan;
    get_msg = 1;
}

double max_dis=0;
std::vector<double> distances;

int main(int argc, char **argv)
{
    std::string node_name = "threeD_lidar_merge";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    if (!nh.getParam("/" + node_name + "/base_frame", base_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'base_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/laser_frame", laser_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'laser_frame'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_left", scan_topic_left))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_left'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/scan_topic_right", scan_topic_right))
    {
        ROS_ERROR("Failed to retrieve parameter 'scan_topic_right'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/new_scan_topic", new_scan_topic))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan_topic'");
        return -1;
    }

    ros::Subscriber sub_left = nh.subscribe(scan_topic_left, 1, scanCallback_left);
    ros::Subscriber sub_right = nh.subscribe(scan_topic_right, 1, scanCallback_right);
    pub = nh.advertise<livox_ros_driver2::CustomMsg>(new_scan_topic, 1);
    ros::Rate rate(50.0);
    while (ros::ok())
    {
        if (!get_msg)
        {
            ros::spinOnce();
            continue;
        }
        auto scan_new = scan_copy_right;
        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;
        scan_new.points.clear();
        for (int i = 0; i < scan_record_left.points.size(); i++){
            scan_new.points.push_back(scan_record_left.points[i]);
            scan_new.points[scan_new.points.size()-1].x -= 0.011;
            scan_new.points[scan_new.points.size()-1].y -= 0.02329;
            scan_new.points[scan_new.points.size()-1].z += 0.04412;
        }
        for (int i = 0; i < scan_record_right.points.size(); i++){
            scan_new.points.push_back(scan_record_right.points[i]);
            scan_new.points[scan_new.points.size()-1].x -= 0.011;
            scan_new.points[scan_new.points.size()-1].y -= 0.02329;
            scan_new.points[scan_new.points.size()-1].z += 0.04412;
        }
        // for(int i=(int)distances.size()-1;i>=0;i--)printf("%lf ",distances[i]);
        // printf("\nmax_dis: %f\n----------------------\n",max_dis);
        scan_new.header.stamp = ros::Time::now();
        scan_new.point_num = scan_new.points.size();
        pub.publish(scan_new);
        get_msg = 0;
        rate.sleep();
    }
    return 0;
}
