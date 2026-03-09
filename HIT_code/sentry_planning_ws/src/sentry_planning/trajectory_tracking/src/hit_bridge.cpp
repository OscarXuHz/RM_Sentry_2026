/**
 * hit_bridge — Converts HIT MPC output (sentry_msgs/slaver_speed) to
 *              geometry_msgs/Twist for the mcu_communicator serial bridge.
 *
 * ─── Two output modes (selected via use_omega_output param) ───
 *
 * [OMNI MODE – default]   (use_omega_output = false)
 *   For the OLD omnidirectional robot.
 *   slaver_speed fields carry: (angle_target=heading, angle_current=lidar_yaw, line_speed=v)
 *   Bridge decomposes v into body-frame vx/vy:
 *     δ = angle_target − angle_current
 *     vx = line_speed · cos(δ)
 *     vy = line_speed · sin(δ)
 *   z_angle = 0  (heading is implicit in vx/vy decomposition;
 *                 the old MCU does NOT use z_angle for heading PID)
 *
 * [OMEGA MODE]              (use_omega_output = true)
 *   For the NEW differential/unicycle robot.
 *   slaver_speed fields carry: (angle_target=ω, angle_current=unused, line_speed=v)
 *   Bridge passes through:
 *     vx      = line_speed   (forward speed)
 *     vy      = 0            (cannot strafe)
 *     z_angle = angle_target (angular velocity ω; MCU does its own heading PID)
 */
#include <ros/ros.h>
#include <sentry_msgs/slaver_speed.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>

ros::Publisher twist_pub;
ros::Publisher arrived_pub;
bool g_use_omega_output = false;

void speedCallback(const sentry_msgs::slaver_speed::ConstPtr& msg)
{
    geometry_msgs::Twist twist;

    if (g_use_omega_output) {
        // ── OMEGA MODE (new robot: vx + ω, cannot strafe) ──
        twist.linear.x  = msg->line_speed;     // forward speed v
        twist.linear.y  = 0.0;                 // no strafing
        twist.linear.z  = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = msg->angle_target;   // angular velocity ω
    } else {
        // ── OMNI MODE (old robot: projected vx/vy, z_angle=0) ──
        double delta = msg->angle_target - msg->angle_current;
        twist.linear.x  = msg->line_speed * std::cos(delta);  // body-frame forward
        twist.linear.y  = msg->line_speed * std::sin(delta);  // body-frame leftward
        twist.linear.z  = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;                                // NOT used by old MCU
    }

    twist_pub.publish(twist);

    // Arrival flag: tracking_node zeroes line_speed when it reaches the goal.
    std_msgs::Bool arrived;
    arrived.data = (std::fabs(msg->line_speed) < 0.01);
    arrived_pub.publish(arrived);

    ROS_DEBUG("[hit_bridge] mode=%s  vx=%.3f vy=%.3f z=%.3f",
              g_use_omega_output ? "omega" : "omni",
              twist.linear.x, twist.linear.y, twist.angular.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hit_bridge");
    ros::NodeHandle nh("~");

    std::string cmd_vel_topic, speed_topic, arrived_topic;
    nh.param<std::string>("cmd_vel_topic",  cmd_vel_topic,  "/cmd_vel");
    nh.param<std::string>("speed_topic",    speed_topic,    "/sentry_des_speed");
    nh.param<std::string>("arrived_topic",  arrived_topic,  "/dstar_status");
    nh.param<bool>("use_omega_output",      g_use_omega_output, false);

    twist_pub   = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    arrived_pub = nh.advertise<std_msgs::Bool>(arrived_topic, 1);

    ros::Subscriber speed_sub = nh.subscribe(speed_topic, 10, speedCallback);

    ROS_INFO("[hit_bridge] mode=%s  %s -> %s  + %s (arrived)",
             g_use_omega_output ? "OMEGA (vx + omega, vy=0)" : "OMNI (vx/vy projected, z_angle=0)",
             speed_topic.c_str(), cmd_vel_topic.c_str(), arrived_topic.c_str());

    ros::spin();
    return 0;
}
