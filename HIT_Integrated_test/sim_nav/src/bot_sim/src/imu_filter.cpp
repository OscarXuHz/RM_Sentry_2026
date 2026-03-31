#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include<string.h>
ros::Publisher imu_pub;

double gravity;
std::string output_frame_id;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 直接将接收到的消息发布到新的topic
    sensor_msgs::Imu imu_msg = *msg;
    if (!output_frame_id.empty()) {
        imu_msg.header.frame_id = output_frame_id;
    }
    imu_msg.linear_acceleration.z *= -gravity;
    imu_msg.linear_acceleration.x *= -gravity;
    imu_msg.linear_acceleration.y *= -gravity;
    double angle_rad = -20 * M_PI / 180.0;
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);
    // 保存原始y/z分量，避免覆盖
    double y0 = imu_msg.linear_acceleration.y;
    double z0 = imu_msg.linear_acceleration.z;
    imu_msg.linear_acceleration.y = y0 * cos_a + z0 * sin_a;
    imu_msg.linear_acceleration.z = (-y0 * sin_a + z0 * cos_a);
    
    // 对角速度进行相同的绕x轴旋转20度操作
    double y_ang = imu_msg.angular_velocity.y;
    double z_ang = imu_msg.angular_velocity.z;
    imu_msg.angular_velocity.y = y_ang * cos_a + z_ang * sin_a;
    imu_msg.angular_velocity.z = -y_ang * sin_a + z_ang * cos_a;
    
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
    nh.param<std::string>("/" + node_name + "/output_frame_id", output_frame_id, std::string(""));
    // 订阅原始IMU数据
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(input_imu_topic, 10, imuCallback);

    // 发布到新的IMU topic
    imu_pub = nh.advertise<sensor_msgs::Imu>(output_imu_topic, 10);

    ros::spin(); // 进入循环，等待回调函数

    return 0;
}