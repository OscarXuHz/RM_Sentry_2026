#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include<string.h>
ros::Publisher imu_pub;

double gravity;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 直接将接收到的消息发布到新的topic
    sensor_msgs::Imu imu_msg = *msg;
    imu_msg.header.frame_id = "aft_mapped";
    imu_msg.linear_acceleration.z *= -gravity;
    imu_msg.linear_acceleration.x *= gravity; 
    imu_msg.linear_acceleration.y *= -gravity;
    // imu_msg.angular_velocity.x *= -1;
    imu_msg.angular_velocity.y *= -1;
    imu_msg.angular_velocity.z *= -1;
    imu_pub.publish(imu_msg);
}

std::string input_imu_topic;
std::string output_imu_topic;
int main(int argc, char **argv) {
    std::string node_name="imu_filter";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    if(!nh.getParam("/"+node_name+"/input_imu_topic", input_imu_topic)){
        ROS_ERROR("Failed to retrieve parameter 'input_imu_topic'");
        return -1;
    }
    if(!nh.getParam("/"+node_name+"/output_imu_topic", output_imu_topic)){
            ROS_ERROR("Failed to retrieve parameter 'output_imu_topic'");
            return -1;
    }
    if(!nh.getParam("/"+node_name+"/gravity", gravity)){
        ROS_ERROR("Failed to retrieve parameter 'gravity'");
        return -1;
    }
    // 订阅原始IMU数据
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(input_imu_topic, 10, imuCallback);

    // 发布到新的IMU topic
    imu_pub = nh.advertise<sensor_msgs::Imu>(output_imu_topic, 10);

    ros::spin(); // 进入循环，等待回调函数

    return 0;
}