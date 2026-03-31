#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <algorithm>
// #include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <list>
#include <sentry_msgs/slaver_speed.h>
#include <sensor_msgs/Imu.h>
#include <sentry_msgs/RobotsHP.h>
#include <sentry_msgs/RobotStatus.h>
#include "KMF.h"
#include "local_planner.h"
#include "visualization_utils.h"
#include <std_msgs/Bool.h>
#include "../include/KMF.h"
#include <numeric>

class tracking_manager
{
public:
    // (Fix 54a) DISPENSE mode removed entirely.
    // Previously: planningType enum with FASTMOTION/DISPENSE.
    // DISPENSE reversed velocity when stuck, but caused feedback loops.
    // Now replaced by simple replan-on-stuck (3s timeout).

    typedef enum {
        red = 0,
        blue,
    } teamColor;
    teamColor sentryColor = teamColor::red;

    double mate_outpost_hp = 1500;

    /* ROS订阅器与发布器 */
    ros::Subscriber vis_debugflag_sub;
    ros::Subscriber gazebo_real_pos_sub;    // 仿真环境中真实位置订阅器
    ros::Subscriber lidar_imu_pos_sub;      // 雷达IMU解算位置订阅器
    ros::Subscriber sentry_speed_lidar;
    ros::Subscriber sentry_wheelyawspeed_feedback;   // 轮速计的回调
    ros::Subscriber robot_command_sub;
    ros::Subscriber robot_status_sub, sentry_HP_sub;

    ros::Publisher robot_cur_yaw_pub;
    ros::Publisher robot_left_speed_pub;
    ros::Publisher robot_right_speed_pub;
    ros::Publisher sentry_speed_pub;
    ros::Publisher sentry_des_pub;
    ros::Publisher cur_position_vis_pub;           // 当前位置可视化发布器
    ros::Publisher robot_speed_yaw_pub;
    ros::Publisher robot_odometry_speed_pub;
    ros::Publisher robot_cur_speed_pub;
    ros::Publisher reference_speed_pub;
    ros::Publisher m_start_point_velocity_pub;
    ros::Publisher m_opt_globalpath_pub;
    ros::Publisher m_imu_acc_pub, m_kal_acc_pub, m_solver_status_pub;
    ros::Publisher global_replan_pub;
    ros::Publisher robot_cur_yaw_deg_pub;

    /* 相关对象 */
    KMF kalman_filter_speedx;
    KMF kalman_filter_speedy;
    KMF kalman_filter_yaw;
    KMF kalman_filter_speed_yaw;

    KMF gazebo_accvel_filter;
    clock_t gazebotime;
    clock_t lasttime;

    ros::Time control_time;

    std::unique_ptr<LocalPlanner> localplanner;
    std::unique_ptr<Vislization> vislization_util;
    std::shared_ptr<GlobalMap> global_map;

    std::string occ_file_path;
    std::string bev_file_path;
    std::string distance_map_file_path;

    double map_resolution;
    double map_inv_resolution;
    double map_x_size;
    double map_y_size;
    double map_z_size;
    Eigen::Vector3d map_lower_point;  /// 地图的左上角和右下角
    Eigen::Vector3d map_upper_point;
    double search_height_min;         /*局部点云可视范围*/
    double search_height_max;
    double search_radius;
    int grid_max_id_x;
    int grid_max_id_y;
    int grid_max_id_z;

    int last_motion_xtl_flag = 0;

    double robot_radius;
    double robot_radius_dash;

    /* 机器人相关状态 */
    Eigen::Vector3d robot_cur_position;
    Eigen::Quaterniond robot_cur_orientation;
    Eigen::Vector3d robot_cur_posture;
    double robot_cur_target_dist;               // 当前与最近目标点的距离
    bool robot_target_arrive_flag;              // 抵达目标位置标志位
    double robot_cur_yaw;                       // 反馈得到的当前的yaw角
    double robot_odometry_speed;
    double robot_speed_yaw;
    double robot_crash_radius;
    double robot_wheel_tread;    // 车轮宽度
    double robot_wheel_speed = 0.0;
    double robot_wheel_yaw = 0.0;
    bool has_wheel_state_ = false;  // becomes true once /slaver/wheel_state is received

    // Fix 44a: Continuous velocity heading for MPC state observation.
    // Avoids ±π wrapping discontinuities that corrupt solver warm-start.
    double velocity_heading_ = 0.0;
    bool velocity_heading_init_ = false;
    double m_reference_speed;
    double target_point_arrive_radius;
    Eigen::Vector2d robot_cur_speed;

    Eigen::Vector2d accleration_imu;
    double line_accleration = 0.0;
    double filter_accleration = 0.0;
    double imu_data_num = 0;
    std::list<double> imu_acc;
    std::vector<int> motion_mode_vec;

    /* 相关标志位 */
    bool isxtl = false;    // 陀螺标志位
    bool in_bridge = false;
    bool use_omega_output_ = false;  // v+omega output mode switch (vs default v+theta)

    int countHP;
    int sentry_HP;
    bool is_attacked;

    bool visualization_flag;
    double speed_init_q;
    double speed_init_r;
    double yaw_init_q;
    double yaw_init_r;

    double last_current_yaw;
    std::vector<int> replan_check_flag;
    bool arrival_goal = true;
    bool replan_now = false;


    /* 轨迹规划相关函数 */
    void init(ros::NodeHandle &nh);
    Eigen::Vector2d getDifferentialModelSpeed(double line_speed, double angular, double robot_wheel_tread);
    void publishMbotOptimalSpeed(Eigen::Vector2d &speed);
    void publishSentryOptimalSpeed(Eigen::Vector4d &speed);
    void checkReplanFlag();
    int checkMotionMode();
//    void disPenseFromNow();
    void lidarSmooth(Eigen::Vector3d position, Eigen::Vector3d velocity, double dt);
    void velocityYawSmooth(double velocity_yaw, double dt);
    void gazeboVelAccSmooth(double speed, double dt);
    void gazeboIMUVelAccSmooth(Eigen::Vector2d &velocity, Eigen::Vector2d & imu_data, double dt);
    /* 回调函数*/

    void rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state);
    void rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state);
    void rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state);
    void rcvGazeboIMUCallback(const sensor_msgs::ImuConstPtr &msg);
    void rcvRobotStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg);
    void rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg);
};
