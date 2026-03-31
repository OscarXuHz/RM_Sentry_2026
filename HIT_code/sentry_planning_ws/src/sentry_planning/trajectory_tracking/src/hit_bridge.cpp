/**
 * hit_bridge — Converts HIT MPC output (sentry_msgs/slaver_speed) to
 *              geometry_msgs/Twist for the mcu_communicator serial bridge.
 *
 * ─── World-frame vx/vy passthrough ───
 *
 *   tracking_manager decomposes speed into world-frame vx/vy.
 *   slaver_speed fields carry: (angle_target=vx, angle_current=vy)
 *   Bridge passes through:
 *     twist.linear.x  = angle_target   (world-frame vx)
 *     twist.linear.y  = angle_current  (world-frame vy)
 */
#include <ros/ros.h>
#include <sentry_msgs/slaver_speed.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>

ros::Publisher twist_pub;
ros::Publisher arrived_pub;

void speedCallback(const sentry_msgs::slaver_speed::ConstPtr& msg)
{
    geometry_msgs::Twist twist;

    // Direct passthrough: tracking_manager already computes world-frame vx/vy
    twist.linear.x  = msg->angle_target;   // world-frame vx
    twist.linear.y  = msg->angle_current;  // world-frame vy
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    twist_pub.publish(twist);

    // Arrival flag: tracking_node zeroes speeds when it reaches the goal.
    std_msgs::Bool arrived;
    arrived.data = (std::fabs(msg->angle_target) < 0.01 &&
                    std::fabs(msg->angle_current) < 0.01);
    arrived_pub.publish(arrived);

    ROS_DEBUG("[hit_bridge] vx=%.3f vy=%.3f",
              twist.linear.x, twist.linear.y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hit_bridge");
    ros::NodeHandle nh("~");

    std::string cmd_vel_topic, speed_topic, arrived_topic;
    nh.param<std::string>("cmd_vel_topic",  cmd_vel_topic,  "/cmd_vel");
    nh.param<std::string>("speed_topic",    speed_topic,    "/sentry_des_speed");
    nh.param<std::string>("arrived_topic",  arrived_topic,  "/dstar_status");

    twist_pub   = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    arrived_pub = nh.advertise<std_msgs::Bool>(arrived_topic, 1);

    ros::Subscriber speed_sub = nh.subscribe(speed_topic, 10, speedCallback);

    ROS_INFO("[hit_bridge] world-frame vx/vy passthrough  %s -> %s  + %s (arrived)",
             speed_topic.c_str(), cmd_vel_topic.c_str(), arrived_topic.c_str());

    ros::spin();
    return 0;
}
