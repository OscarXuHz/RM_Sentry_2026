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

bool get_msg_left = 0;
bool get_msg_right = 0;
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
    get_msg_left = 1;
}
void scanCallback_right(const livox_ros_driver2::CustomMsg &scan)
{
    scan_copy_right = scan;
    get_msg_right = 1;
}

double max_dis=0;
std::vector<double> distances;

bool filter(double x, double y, double z){
    double dx = x + 0.13388;
    double dy = y + 0.11369;
    return dx*dx + dy*dy >= 0.35 * 0.35;
}
int main(int argc, char **argv)
{
    std::string node_name = "threeD_lidar_merge_pointcloud";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

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
    pub = nh.advertise<sensor_msgs::PointCloud2>(new_scan_topic, 1);
    ros::Rate rate(50.0);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    while (ros::ok())
    {
        if (!get_msg_left||!get_msg_right)
        {
            ros::spinOnce();
            continue;
        }
        auto scan_new = scan_copy_right;
        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;
        pcl_cloud.points.clear();
        for (int i = 0; i < scan_record_left.points.size(); i++){
            if(!filter(scan_record_left.points[i].x, scan_record_left.points[i].y, scan_record_left.points[i].z))continue;
            double x = scan_record_left.points[i].x - 0.011;
            double y = -scan_record_left.points[i].y + 0.02329;
            double z = -scan_record_left.points[i].z - 0.04412;
            pcl_cloud.points.push_back(pcl::PointXYZ(x, y, z));
        }
        for (int i = 0; i < scan_record_right.points.size(); i++){
            if(!filter(scan_record_right.points[i].x, scan_record_right.points[i].y, scan_record_right.points[i].z))continue;
            double x = scan_record_right.points[i].x - 0.011;
            double y = -scan_record_right.points[i].y + 0.02329;
            double z = -scan_record_right.points[i].z - 0.04412;
            pcl_cloud.points.push_back(pcl::PointXYZ(x, y, z));
        }
        // for(int i=(int)distances.size()-1;i>=0;i--)printf("%lf ",distances[i]);
        // printf("\nmax_dis: %f\n----------------------\n",max_dis);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pcl_cloud, output);
        output.header.frame_id = laser_frame; // replace with your frame id
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        // ---
        // printf("Published new scan\n");
        get_msg_left = get_msg_right = 0;
        rate.sleep();
    }
    return 0;
}
