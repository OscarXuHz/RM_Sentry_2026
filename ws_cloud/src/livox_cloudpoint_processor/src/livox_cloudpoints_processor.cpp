#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "DB-scan.cpp"
#include "process_invisibility.cpp" 
#include "dilation_grid.cpp"
#include <pcl/filters/voxel_grid.h>
//Include for compute normals
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/thread/thread.hpp>
// Include for testing
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>

//parameters given by the launch file
double first_RADIUS, second_RADIUS, max_height,start_height;

double slope_1,slp_first_RADIUS, height_1;
double slope_2, slp_second_RADIUS, height_2;
double slope_3,slp_third_RADIUS, height_3;

double max_dis = 0;
std::vector<double> distances;

bool get_msg_left = 0;
bool get_msg_right = 0;
std::string base_frame;
std::string laser_frame;
std::string scan_topic_left;
std::string scan_topic_right;
std::string new_scan_topic;
std::string new_scan_topic_2d;
std::string filted_topic_3d;
std::string dilated_topic_2d;

ros::Publisher pub;//initialize the publisher
livox_ros_driver2::CustomMsg scan_copy_left;//receiving the message from the livox
livox_ros_driver2::CustomMsg scan_copy_right;
// geometry_msgs::TransformStamped transformStamped;//initialize the transformStamped

//callback function for the livox
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
//function to filter the points
bool ispoint (double nx, double ny, double z, double nI)
{
    nx += 0.011;
    ny -= -0.19495+0.02329;
    if (nx*nx+ny*ny <= first_RADIUS*first_RADIUS){
        return 0;
    }
    if (nx*nx+ny*ny <= second_RADIUS*first_RADIUS){
        if (0.3<=z && z<=0.35){//what's the reference of the param?
            distances.push_back(nx*nx+ny*ny);
            max_dis = std::max(max_dis, nx*nx+ny*ny);
        }
        return z<= start_height;
    }
    if(nx*nx+ny*ny <=slp_first_RADIUS * slp_first_RADIUS){
        double dis=sqrt(nx*nx+ny*ny)-second_RADIUS;
        return z<=std::min(max_height,dis*slope_1+start_height);//max_height for security
    }
    
    if(nx*nx+ny*ny <=slp_second_RADIUS * slp_second_RADIUS){
        double dis=sqrt(nx*nx+ny*ny)-slp_first_RADIUS;
        return z<=std::min(height_1,dis*slope_2+max_height);
    }

    double dis=sqrt(nx*nx+ny*ny)-slp_second_RADIUS;
    return z<=std::min(height_2,dis*slope_3+height_1);
     return 1;
    //add judgement for the indensity
}

void down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f,0.05f,0.05f);
    sor.filter(*cloud_filtered);
}

void statistical_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
}

int main (int argc, char **argv)
{
    std::string node_name = "threeD_lidar_filter_pointcloud";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    double dilation_radius;
    if (!nh.getParam("/" + node_name + "/dilation_radius", dilation_radius)) {
        ROS_ERROR("Failed to retrieve parameter 'dilation_radius'");
        return -1;
    }
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
    if (!nh.getParam("/" + node_name + "/first_RADIUS", first_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'first_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/second_RADIUS", second_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'second_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/max_height", max_height))
    {
        ROS_ERROR("Failed to retrieve parameter 'max_height'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/start_height", start_height))
    {
        ROS_ERROR("Failed to retrieve parameter 'start_height'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_1", slope_1))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_1'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_2", slope_2))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_2'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slope_3", slope_3))
    {
        ROS_ERROR("Failed to retrieve parameter 'slope_3'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_first_RADIUS", slp_first_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_first_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_second_RADIUS", slp_second_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_second_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/slp_third_RADIUS", slp_third_RADIUS))
    {
        ROS_ERROR("Failed to retrieve parameter 'slp_third_RADIUS'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/new_scan_topic_2d", new_scan_topic_2d))
    {
        ROS_ERROR("Failed to retrieve parameter 'new_scan_topic_2d'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/filted_topic_3d", filted_topic_3d))
    {
        ROS_ERROR("Failed to retrieve parameter 'filted_topic_3d'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/dilated_topic_2d", dilated_topic_2d))
    {
        ROS_ERROR("Failed to retrieve parameter 'dilated_topic_2d'");
        return -1;
    }

    ros::Subscriber sub_left = nh.subscribe(scan_topic_left, 1, scanCallback_left);
    ros::Subscriber sub_right = nh.subscribe(scan_topic_right, 1, scanCallback_right);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>(new_scan_topic, 1);//original pointcloud
    ros::Publisher pub3 = nh.advertise<nav_msgs::OccupancyGrid>(new_scan_topic_2d, 1);
    ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2>(filted_topic_3d, 1);//filtered
    ros::Publisher pub5 = nh.advertise<nav_msgs::OccupancyGrid>(dilated_topic_2d, 1);
    ros::Rate rate(50.0);

    while (ros::ok())
    {
        if (!get_msg_left || !get_msg_right)
        {ros::spinOnce();
            // ROS_INFO("waiting for the message");
            continue;
        }
        
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud; 
        pcl::PointCloud<pcl::PointXYZI> origin_pcl_cloud; 
        auto scan_record_left = scan_copy_left;
        auto scan_record_right = scan_copy_right;

        for (int i = 0; i < scan_record_left.points.size(); i++)
        {
            double x = scan_record_left.points[i].x - 0.011;
            double y = -scan_record_left.points[i].y - 0.02329;
            double z = -scan_record_left.points[i].z + 0.04412;
            // double y = scan_record_left.points[i].y + 0.02329;
            // double z = scan_record_left.points[i].z - 0.04412;
            double intensity = scan_record_left.points[i].reflectivity;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
            {
                continue;
            }
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity; 
            origin_pcl_cloud.points.push_back(point);
        }
        for (int i = 0; i < scan_record_right.points.size(); i++)
        {
            double x = scan_record_right.points[i].x - 0.011;
            double y = -scan_record_right.points[i].y - 0.02329;
            double z = -scan_record_right.points[i].z + 0.04412;
            // double y = scan_record_right.points[i].y + 0.02329;
            // double z = scan_record_right.points[i].z - 0.04412;
            double intensity = scan_record_right.points[i].reflectivity;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
            {
                continue;
            }
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity; 
            origin_pcl_cloud.points.push_back(point);
        }
        // print total size of points
        // ROS_INFO("Total points: %zu", scan_record_left.points.size() + scan_record_right.points.size());
        for (int i = 0; i < scan_record_left.points.size(); i++)
        {
            double x = scan_record_left.points[i].x - 0.011;
            double y = -scan_record_left.points[i].y - 0.02329;
            double z = -scan_record_left.points[i].z + 0.04412;
            // double y = scan_record_left.points[i].y + 0.02329;
            // double z = scan_record_left.points[i].z - 0.04412;
            double intensity = scan_record_left.points[i].reflectivity;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
            {
                continue;
            }
            if (ispoint(x,y,-z,intensity))
            {
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity; 
                pcl_cloud.points.push_back(point);
            }
        }
        for (int i = 0; i < scan_record_right.points.size(); i++)
        {
            double x = scan_record_right.points[i].x - 0.011;
            double y = -scan_record_right.points[i].y - 0.02329;
            double z = -scan_record_right.points[i].z + 0.04412;
            // double y = scan_record_right.points[i].y + 0.02329;
            // double z = scan_record_right.points[i].z - 0.04412;
            double intensity = scan_record_right.points[i].reflectivity;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
            {
                continue;
            }
            if (ispoint(x,y,-z,intensity))
            {
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity; 
                pcl_cloud.points.push_back(point);
            }
        }
        // print the size of the filtered points
        // ROS_INFO("Filtered points: %zu", pcl_cloud.points.size());

        // sensor_msgs::PointCloud2 output1;
        // pcl::toROSMsg(pcl_cloud, output1);
        // output1.header.frame_id = laser_frame; // replace with your frame id
        // output1.header.stamp = ros::Time::now();
        // pub2.publish(output1);

        ros::Time start_time = ros::Time::now();//TimeTest Start

        //Down Sampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(pcl_cloud, *pcl_cloud_xyz);
        
        // DBSCAN dbscan(0.2, 5); // radius, minPts
        // dbscan.run(pcl_cloud_xyz);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_xyz_no_intensity(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::copyPointCloud(*pcl_cloud_xyz, *temp_cloud);
        //pcl::copyPointCloud(*pcl_cloud_xyz, *pcl_cloud_xyz_no_intensity);
        down_sampling(pcl_cloud_xyz, temp_cloud);
        statistical_removal(temp_cloud, pcl_cloud_xyz_no_intensity);
        // ROS_INFO("Filtered points2: %zu", pcl_cloud_xyz_no_intensity->points.size());

        // ros::Duration filter_duration = end_time - start_time;
        // ROS_INFO("clustering took %f seconds", filter_duration.toSec());
        // ROS_INFO("Filtered points: %zu", pcl_cloud_xyz_no_intensity->points.size());
        ros::Time end_time = ros::Time::now();
        //Find Normals
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        n.setInputCloud(pcl_cloud_xyz_no_intensity);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.setNumberOfThreads(8);
        n.compute(*normals);

        //Visualization Test
        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->setBackgroundColor(0.3, 0.3, 0.3);
        // viewer->addText("faxian",10,10,"text");
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pcl_cloud_xyz_no_intensity, 0, 255, 0);
        // viewer->addCoordinateSystem(0.1);
        // viewer->addPointCloud<pcl::PointXYZ>(pcl_cloud_xyz_no_intensity, single_color, "cloud");
        
        // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pcl_cloud_xyz_no_intensity, normals, 1, 0.05, "normals");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(100);
        //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        // }

        //Filter the points with the normals vertical to XOY

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < pcl_cloud_xyz_no_intensity->points.size(); ++i) {
            const auto& normal = normals->points[i];
            // 计算法向量与 z 轴的夹角
            float angle = std::acos(normal.normal_z / std::sqrt(normal.normal_x * normal.normal_x + normal.normal_y * normal.normal_y + normal.normal_z * normal.normal_z));
            // 如果夹角不接近 90 度，则保留该点
            if (std::abs(angle - M_PI / 2) < M_PI/4) { // 0.1 弧度的容差
                filtered_cloud->points.push_back(pcl_cloud_xyz_no_intensity->points[i]);
                // ROS_INFO("angle: %f", angle);
            }else{
                // ROS_INFO("angle: %f, x: %f, y: %f, z: %f", angle, pcl_cloud_xyz_no_intensity->points[i].x, pcl_cloud_xyz_no_intensity->points[i].y, pcl_cloud_xyz_no_intensity->points[i].z);
            }
        }

        // 将 filtered_cloud 转换为 pcl::PointCloud<pcl::PointXYZI>
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*filtered_cloud, *filtered_cloud_xyzi);
        end_time = ros::Time::now();
        // ROS_INFO("Filtered points: %zu", filtered_cloud_xyzi->points.size());

        // DBSCAN dbscan(0.2, 5); // radius, minPts
        // dbscan.run(filtered_cloud_xyzi);
        
        // ROS_INFO("Clustered points: %zu", filtered_cloud_xyzi->points.size());

        // TimeTest End
        
        ros::Duration filter_duration = end_time - start_time;
        // ROS_INFO("Filtering took %f seconds", filter_duration.toSec());
        
        // Debug: Print cloud sizes
        // ROS_INFO("Original cloud points: %zu", pcl_cloud.points.size());
        // ROS_INFO("Filtered cloud points: %zu", pcl_cloud_xyz->points.size());

        // Publish original cloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(origin_pcl_cloud, output);
        output.header.frame_id = laser_frame; // replace with your frame id
        output.header.stamp = ros::Time::now();
        pub2.publish(output);

        // Publish filtered cloud
        sensor_msgs::PointCloud2 output_filtered;
        pcl::toROSMsg(*filtered_cloud_xyzi, output_filtered);
        output_filtered.header.frame_id = laser_frame;
        output_filtered.header.stamp = ros::Time::now();
        pub4.publish(output_filtered);

        
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud_2d;
        for (const auto& point : *filtered_cloud_xyzi) {
            pcl::PointXYZI point_2d;
            point_2d.x = point.x;
            point_2d.y = point.y;
            point_2d.z = 0.0; // 将z坐标设置为0，转换为二维点云
            point_2d.intensity = point.intensity;
            pcl_cloud_2d.points.push_back(point_2d);
        }
        
        nav_msgs::OccupancyGrid occupancy_grid;
        occupancy_grid.header.frame_id = laser_frame;
        occupancy_grid.header.stamp = ros::Time::now();
        occupancy_grid.info.resolution = 0.05; // 栅格分辨率
        occupancy_grid.info.width = 200; // 栅格地图宽度（单位：栅格数）
        occupancy_grid.info.height = 200; // 栅格地图高度（单位：栅格数）
        occupancy_grid.info.origin.position.x = -5.0; // 地图原点的x坐标
        occupancy_grid.info.origin.position.y = -5.0; // 地图原点的y坐标
        occupancy_grid.info.origin.position.z = 0.0;
        occupancy_grid.info.origin.orientation.w = 1.0;

        occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, 0);

        double center_x = occupancy_grid.info.origin.position.x + occupancy_grid.info.width * occupancy_grid.info.resolution / 2.0;
        double center_y = occupancy_grid.info.origin.position.y + occupancy_grid.info.height * occupancy_grid.info.resolution / 2.0;
        
        for (const auto& point : pcl_cloud_2d.points) {
            int x_index = static_cast<int>((point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution);
            int y_index = static_cast<int>((point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution);

            if (x_index >= 0 && x_index < occupancy_grid.info.width && y_index >= 0 && y_index < occupancy_grid.info.height) {
                int index = y_index * occupancy_grid.info.width + x_index;
                occupancy_grid.data[index] = 100; // 设置占据栅格的值（0-100）
            }
        }
        
        //pub3.publish(occupancy_grid);

        start_time = ros::Time::now();
        dilateOccupancyGrid(occupancy_grid, dilation_radius);
        end_time = ros::Time::now();

        ros::Duration dilation_duration = end_time - start_time;
        // ROS_INFO("Dilation took %f seconds", dilation_duration.toSec());

        start_time = ros::Time::now();
        processVisibility(occupancy_grid);
        end_time = ros::Time::now();

        ros::Duration invisibility_duration = end_time - start_time;
        // ROS_INFO("Invisibility took %f seconds", invisibility_duration.toSec());

        pub5.publish(occupancy_grid);

        get_msg_left = 0;
        get_msg_right = 0;
        rate.sleep();


    }
    return 0;
}
