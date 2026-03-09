#ifndef RM_REPLAN_H
#define RM_REPLAN_H

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int64.h>
#include <sentry_msgs/slaver_speed.h>
#include <sentry_msgs/RobotsHP.h>
#include <sentry_msgs/RobotStatus.h>

#include <cmath>
#include <queue>
#include <algorithm>

#include "plan_manager.h"
#include "visualization_utils.h"

class ReplanFSM
{
public:

    /** 规划func **/
    std::unique_ptr<planner_manager> plannerManager;
    std::unique_ptr<Vislization> vislization;

    /* 机器人相关位置，速度，yaw角速度，姿态状态 */
    Eigen::Vector3d robot_cur_position;

    Eigen::Quaterniond robot_cur_orientation;
    Eigen::Vector3d robot_cur_speed;
    double robot_line_speed;
    double robot_angular;
    double robot_cur_yaw;
    double robot_wheel_speed;
    double robot_wheel_yaw;

    int countHP;
    int sentry_HP;

    Eigen::Vector3d start_pt;
    Eigen::Vector3d final_goal;

    typedef enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        WAIT_TOPO_TRAJ
    } planningStatus;

    typedef enum PLANNING_TYPE
    {
        TEAMFLIGHT = 1,
        SAFETEAMFLIGHT = 2,
        FASTMOTION = 3
    } planningType;

    planningStatus sentryStatus;
    planningType planningMode;

    int m_target_type, decision_mode = 0;
    int tracking_mode = 0;
    bool have_target, have_odom, have_attack_target;
    bool is_attacked;
    bool planning_succeed;
    bool visualization_flag;
    bool replan_flag = false;  // 判断重规划是否成功

    double robot_radius;
    double robot_radius_dash;

    ros::Timer exeTimer, decisionTimer, planningTimer;  //轨迹规划Timer
    ros::Subscriber odom_sub, odom_gazebo_sub, slaver_sub, rviz_despt_sub;
    ros::Subscriber sentry_HP_sub, sentry_status_sub, robot_command_sub, enemy_num_sub, hp_sub;
    ros::Subscriber replan_flag_sub;
    ros::Publisher poly_traj_pub, grid_map_vis_pub, optimized_path_vis_pub, cur_position_vis_pub;
    ros::Publisher global_planning_result_pub, reference_path_vis_pub, attack_engineer_pub, local_grid_map_vis_pub;
    ros::Publisher xtl_mode_pub, target_point_pub;

    void init(ros::NodeHandle &nh);
    /* 回调函数*/
    void rcvWaypointsCallback(const geometry_msgs::PointStampedConstPtr &wp);  /// 接受手动指定点
    void rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state);
    void rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state);
    void rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state);
    void rcvSentryStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg);
    void rcvPlanningModeCallback(const std_msgs::Int64ConstPtr &msg);
    void rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg);
    void rcvHPCallback(const sentry_msgs::RobotsHPConstPtr &msg);

    void execFSMCallback(const ros::TimerEvent &e);
    void checkReplanCallback(const std_msgs::BoolConstPtr &msg);

};

#endif