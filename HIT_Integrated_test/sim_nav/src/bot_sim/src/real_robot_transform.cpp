#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
const float PI = 3.14159265358979323846;

std::string v_frame;
std::string g_frame;
std::string odom_frame;
tf2::Quaternion qhdl;
bool received_msg = 0;
tf2_ros::Buffer tfBuffer;

void listenTransform()
{
    geometry_msgs::TransformStamped transformStamped;

    try {
        transformStamped = tfBuffer.lookupTransform("aft_mapped", "map", ros::Time(0), ros::Duration(3.0));
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
        );
        qhdl = q;
        tf2::Matrix3x3 m(qhdl);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        qhdl.setRPY(roll, pitch, 0);
        // qhdl = qhdl.inverse();
        ROS_INFO("Quaternion_hdl: x=%f, y=%f, z=%f, w=%f", qhdl.x(), qhdl.y(), qhdl.z(), qhdl.w());
        received_msg = 1;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}


int main(int argc, char** argv){
    std::string node_name = "real_robot_transform";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string gimbal_frame;
    if (!nh.getParam("/"+node_name+"/gimbal_frame", gimbal_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'gimbal_frame'");
        return -1;
    }
    std::string _3DLidar_frame;
    if (!nh.getParam("/"+node_name+"/_3DLidar_frame", _3DLidar_frame))
    {
        ROS_ERROR("Failed to retrieve parameter '_3DLidar_frame'");
        return -1;
    }
    if (!nh.getParam("/"+node_name+"/g_frame", g_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'g_frame'");
        return -1;
    }
    if (!nh.getParam("/"+node_name+"/v_frame", v_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'v_frame'");
        return -1;
    }
    if (!nh.getParam("/"+node_name+"/odom_frame", odom_frame))
    {
        ROS_ERROR("Failed to retrieve parameter 'v_frame'");
        return -1;
    }

    tf2_ros::TransformBroadcaster broadcaster;

    tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::Subscriber sub = nh.subscribe("/aft_mapped_to_init", 1000, odometryCallback);

    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.frame_id = _3DLidar_frame;
    transformStamped1.child_frame_id = gimbal_frame;
    transformStamped1.transform.translation.x = -0.011;
    transformStamped1.transform.translation.y = -0.19495+0.02329;
    transformStamped1.transform.translation.z = 0;
    transformStamped1.transform.rotation.x = 0;
    transformStamped1.transform.rotation.y = 0;
    transformStamped1.transform.rotation.z = 0;
    transformStamped1.transform.rotation.w = 1;
    tf2::Quaternion qx;
    qx.setRPY(PI, 0, 0);
    tf2::Quaternion qz;
    qz.setRPY(0, 0, 45 * PI/180);

    ros::Rate rate(20.0);
    while (nh.ok()){
        listenTransform();
        if(received_msg){
            transformStamped1.header.stamp = ros::Time::now();
            auto q =  qhdl;
            // transformStamped1.transform.rotation.x = q.x();
            // transformStamped1.transform.rotation.y = q.y();
            // transformStamped1.transform.rotation.z = q.z();
            // transformStamped1.transform.rotation.w = q.w();
            broadcaster.sendTransform(transformStamped1);
            received_msg = 0;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
