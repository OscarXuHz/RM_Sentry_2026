//
// Created by zzt on 23-9-26.
//

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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int64.h>
#include <trajectory_generation/trajectoryPoly.h>

#include <cmath>
#include <queue>
#include <algorithm>

#include "../include/replan_fsm.h"


void ReplanFSM::init(ros::NodeHandle &nh)
{
    sentryStatus = planningStatus::INIT;
    planningMode = planningType::FASTMOTION;
    have_target = false;
    have_attack_target = false;
    planning_succeed = false;
    visualization_flag = true;
    is_attacked = false;
    have_odom = false;

    plannerManager.reset(new planner_manager);
    plannerManager->init(nh);

    vislization.reset(new Vislization);
    vislization->init(nh);
    nh.param("trajectory_generator/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("trajectory_generator/robot_radius", robot_radius, 0.35);

    exeTimer = nh.createTimer(ros::Duration(0.03), &ReplanFSM::execFSMCallback, this);
    replan_flag_sub = nh.subscribe("/replan_flag", 1, &ReplanFSM::checkReplanCallback, this);

    odom_sub = nh.subscribe("/odometry_imu", 1, &ReplanFSM::rcvLidarIMUPosCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    odom_gazebo_sub = nh.subscribe("/gazebo/model_states", 1, &ReplanFSM::rcvGazeboRealPosCallback, this);

    slaver_sub = nh.subscribe("/slaver/wheel_state", 1, &ReplanFSM::rcvSentryWheelSpeedYawCallback, this);
    rviz_despt_sub = nh.subscribe("/clicked_point", 1, &ReplanFSM::rcvWaypointsCallback, this);

    sentry_HP_sub = nh.subscribe("/slaver/robot_HP", 1, &ReplanFSM::rcvSentryHPJudgeCallback, this);
    hp_sub = nh.subscribe("/referee/robots_hp", 1, &ReplanFSM::rcvHPCallback, this);
    sentry_status_sub = nh.subscribe("/slaver/robot_status", 1, &ReplanFSM::rcvSentryStatusCallback, this);

    target_point_pub = nh.advertise<geometry_msgs::Point>("/target_result", 1);
    xtl_mode_pub = nh.advertise<std_msgs::Bool>("/xtl_flag", 1);
    global_planning_result_pub = nh.advertise<trajectory_generation::trajectoryPoly>("global_trajectory", 1);
    grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    local_grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("local_grid_map_vis", 1);
}

void ReplanFSM::rcvWaypointsCallback(const geometry_msgs::PointStampedConstPtr &wp)
{
    /**
     * @brief rviz目标位置
     * @param
     */
    /* 读出三维目标值 */
    Eigen::Vector3d target_pt;
    target_pt << wp->point.x,
            wp->point.y,
            wp->point.z;

    target_pt.z() = 0;
    final_goal = target_pt;
    have_target = true;
    sentryStatus = planningStatus::GEN_NEW_TRAJ;
    ROS_INFO("[FSM Get Target] generate trajectory!]");
    std::cout<<"target_pt: "<<target_pt.x() <<", "<<target_pt.y()<<", "<<target_pt.z()<<std::endl;

    Eigen::Vector3i goal_idx = plannerManager->global_map->coord2gridIndex(final_goal);
    if (plannerManager->global_map->isOccupied(goal_idx, false)) {
        Eigen::Vector3i goal_neigh_idx;
        if (plannerManager->astar_path_finder->findNeighPoint(goal_idx, goal_neigh_idx, 2)) {
            goal_idx = goal_neigh_idx;
        }
    }
    final_goal = plannerManager->global_map->gridIndex2coord(goal_idx);
    final_goal.z() = plannerManager->global_map->getHeight(goal_idx.x(), goal_idx.y());
}

void ReplanFSM::rcvGazeboRealPosCallback(const gazebo_msgs::ModelStatesConstPtr &state)
{
    /**
     * @brief 仿真环境定位反馈位置
     * @param state:gazebo返回数据
     */
    /* 实时位置 */
    int robot_namespace_id = 0;
    for(int i = 0;i < state->name.size();i++){
        if(state->name[i] == "mbot"){
            robot_namespace_id = i;
            break;
        }
    }

    robot_cur_position(0) = state->pose[robot_namespace_id].position.x;
    robot_cur_position(1) = state->pose[robot_namespace_id].position.y;
    robot_cur_position(2) = state->pose[robot_namespace_id].position.z;

    /* 使用转换公式获取实时姿态(yaw取-pi~pi) */
    double x = state->pose[robot_namespace_id].orientation.x;
    double y = state->pose[robot_namespace_id].orientation.y;
    double z = state->pose[robot_namespace_id].orientation.z;
    double w = state->pose[robot_namespace_id].orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    robot_cur_yaw = atan2f(siny_cosp, cosy_cosp);

    /* 重设规划起始位置 */
    start_pt(0) = robot_cur_position(0);
    start_pt(1) = robot_cur_position(1);
    start_pt(2) = robot_cur_position(2);

    /* 将点云坐标映射为栅格ID(栅格ID从地图左下角算起) */
    Eigen::Vector3i start_idx = plannerManager->global_map->coord2gridIndex(start_pt);
    if (plannerManager->global_map->isOccupied(start_idx, false)) {
        Eigen::Vector3i start_neigh_idx;
        if (plannerManager->astar_path_finder->findNeighPoint(start_idx, start_neigh_idx, 2)) {
            start_idx = start_neigh_idx;
        }
    }
    Eigen::Vector3d start_temp = plannerManager->global_map->gridIndex2coord(start_idx);
    start_pt.x() = start_temp.x();
    start_pt.y() = start_temp.y();  /// 保留初始z轴高度

    robot_cur_speed(0) = state->twist[robot_namespace_id].linear.x;
    robot_cur_speed(1) = state->twist[robot_namespace_id].linear.y;
    robot_cur_speed(2) = 0;
    have_odom = true;
}

void ReplanFSM::rcvLidarIMUPosCallback(const nav_msgs::OdometryConstPtr &state)
{
    /**
     * @brief 实车定位反馈位置
     * @param state:定位返回数据
     */
    /* 实时位置 */
    geometry_msgs::Pose pose;
    pose = state->pose.pose;
    robot_cur_position(0) = pose.position.x;
    robot_cur_position(1) = pose.position.y;
    robot_cur_position(2) = pose.position.z - 0.2;

    /* 使用转换公式获取实时姿态(yaw取-pi~pi) 这里的yaw轴姿态是雷达姿态，这里因为雷达固连在底盘上所以雷达的姿态就等于底盘姿态 */
    double x = pose.orientation.x;
    double y = pose.orientation.y;
    double z = pose.orientation.z;
    double w = pose.orientation.w;
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);

    /* 重设规划起始位置 */
    start_pt(0) = robot_cur_position(0);
    start_pt(1) = robot_cur_position(1);
    start_pt(2) = robot_cur_position(2);

    Eigen::Vector3i start_idx = plannerManager->global_map->coord2gridIndex(start_pt);
    if (plannerManager->global_map->isOccupied(start_idx, false)) {
        Eigen::Vector3i start_neigh_idx;
        if (plannerManager->astar_path_finder->findNeighPoint(start_idx, start_neigh_idx, 2)) {
            start_idx = start_neigh_idx;
        }
    }
    Eigen::Vector3d start_temp = plannerManager->global_map->gridIndex2coord(start_idx);
    start_pt.x() = start_temp.x();
    start_pt.y() = start_temp.y();  /// 保留初始z轴高度

    have_odom = true;
}


void ReplanFSM::rcvSentryStatusCallback(const sentry_msgs::RobotStatusConstPtr &msg)
{
    /**
     * @brief 红蓝方判断
     * @param msg:裁判系统返回数据
     */
    if(msg->id == 7){
        plannerManager->sentryColor = plannerManager->teamColor::red;
    }
    else if (msg->id == 107){
        plannerManager->sentryColor = plannerManager->teamColor::blue;
    }
    ROS_WARN("[FSM Status] sentry_color: %d",plannerManager->sentryColor);
}

void ReplanFSM::rcvHPCallback(const sentry_msgs::RobotsHPConstPtr &msg)
{
    static int last_hp = 400;
    int current_hp = (plannerManager->sentryColor == planner_manager::teamColor::red)
                         ? msg->red_sentry_hp
                         : msg->blue_sentry_hp;

    if (last_hp - current_hp > 20) {
        std_msgs::Bool xtl_msg;
        xtl_msg.data = true;
        xtl_mode_pub.publish(xtl_msg);
    }

    last_hp = current_hp;
}

void ReplanFSM::rcvSentryHPJudgeCallback(const sentry_msgs::RobotsHPConstPtr &msg)
{
    /**
     * @brief 血量判断
     * @param msg:裁判系统返回数据
     */
    countHP ++;   /// 血量返回计数
    int current_HP;
    if(plannerManager->sentryColor == plannerManager->teamColor::red){  // 红蓝方判断
        current_HP = msg->red_sentry_hp;
    }else{
        current_HP = msg->blue_sentry_hp;
    }

    if(plannerManager->sentry_HP == 0){  /// 初始化当前的烧饼血量
        sentry_HP = current_HP;
    }

    if((plannerManager->sentry_HP - current_HP) >= 2){ /// 血量减少超过2，说明被攻击，开始计数并转换标志位
        countHP = 0;
        is_attacked = true;
        plannerManager->isxtl = true;
        ROS_ERROR("[FSM SentryHP] is_attacked!!");
    }else if(countHP > 10 && (plannerManager->sentry_HP == current_HP)){
        is_attacked = false;
        plannerManager->isxtl = false;
    }
    plannerManager->sentry_HP = current_HP;
    return;
}

void ReplanFSM::rcvSentryWheelSpeedYawCallback(const geometry_msgs::Vector3ConstPtr &state)
{
    /**
     * @brief 轮速计返回数据
     * @param state:电控返回数据，包括速度和行进角度
     */
    if(abs(state->x) < 10.0){
        robot_wheel_speed = state->x;
    }
    robot_wheel_yaw = state->y * M_PI / 180;

    int k = robot_wheel_yaw / (2 * M_PI);
    robot_wheel_yaw = robot_wheel_yaw - k * (2 * M_PI);

    int j = robot_wheel_yaw / M_PI;
    robot_wheel_yaw = robot_wheel_yaw - j * (2 * M_PI);

    if(have_odom){
        robot_cur_yaw = robot_wheel_yaw;
        robot_cur_speed(0) = robot_wheel_speed * cos(robot_wheel_yaw);
        robot_cur_speed(1) = robot_wheel_speed * sin(robot_wheel_yaw);
        robot_cur_speed(2) = 0;
    }
}

void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    /**
     * @brief 规划定时回调
     * @param
     */
    geometry_msgs::Point target_vis;
    target_vis.x = final_goal.x();
    target_vis.y = final_goal.y();
    target_vis.z = final_goal.z();
    target_point_pub.publish(target_vis);
    plannerManager->astar_path_finder->visLocalGridMap(*(plannerManager->global_map->m_local_cloud), true);
    vislization->visAstarPath(plannerManager->astar_path);
    vislization->visOptimizedPath(plannerManager->final_path);
    vislization->visOptGlobalPath(plannerManager->ref_trajectory);
    vislization->visFinalPath(plannerManager->optimized_path);
    vislization->visCurPosition(robot_cur_position);
    vislization->visTopoPointGuard(plannerManager->topo_prm->m_graph);
    vislization->visTopoPointConnection(plannerManager->topo_prm->m_graph);

    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();
    if(sentryStatus == planningStatus::INIT){
        if(!have_odom){
            return;
        }
        sentryStatus = planningStatus::WAIT_TARGET;
    }

    if(sentryStatus == planningStatus::WAIT_TARGET){
        if(!have_target){
            return;
        }
        sentryStatus = planningStatus::GEN_NEW_TRAJ;
    }

    if(sentryStatus == planningStatus::GEN_NEW_TRAJ){
        if(plannerManager->pathFinding(start_pt, final_goal, robot_cur_speed)){
            sentryStatus = planningStatus::EXEC_TRAJ;
            trajectory_generation::trajectoryPoly global_path;
            double desired_time = 0.0;
            global_path.motion_mode = decision_mode;
            for(int i = 0; i < plannerManager->reference_path->m_trapezoidal_time.size(); i++){
                desired_time += plannerManager->reference_path->m_trapezoidal_time[i];

                global_path.duration.push_back(plannerManager->reference_path->m_trapezoidal_time[i]);
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 0)); // d
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 1));
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 2));
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 3));

                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 0)); // d
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 1));
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 2));
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 3));
            }
            global_planning_result_pub.publish(global_path);
            ROS_INFO("[FSM] trajectory generate time: %f", (ros::Time::now() - t1).toSec());
            ROS_INFO("[FSM] trajectory desired time: %f", desired_time);
        }
        else
        {
            ROS_ERROR("[FSM] generate trajectory failed");
            return;
        }
    }

    if(sentryStatus == planningStatus::EXEC_TRAJ){
//        ROS_WARN("-------- generated trajectory is safe! --------");
    }

    if(sentryStatus == planningStatus::REPLAN_TRAJ){
        if(plannerManager->replanFinding(robot_cur_position, final_goal, robot_cur_speed)){
            sentryStatus = planningStatus::EXEC_TRAJ;
            trajectory_generation::trajectoryPoly global_path;
            global_path.motion_mode = decision_mode;
            double desired_time = 0.0;
            for(int i = 0; i < plannerManager->reference_path->m_trapezoidal_time.size(); i++){
                desired_time += plannerManager->reference_path->m_trapezoidal_time[i];
                global_path.duration.push_back(plannerManager->reference_path->m_trapezoidal_time[i]);
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 0)); // d
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 1));
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 2));
                global_path.coef_x.push_back(plannerManager->reference_path->m_polyMatrix_x(i, 3));

                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 0)); // d
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 1));
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 2));
                global_path.coef_y.push_back(plannerManager->reference_path->m_polyMatrix_y(i, 3));
            }
            global_planning_result_pub.publish(global_path);
            ROS_INFO("[FSM] trajectory generate time: %f", (ros::Time::now() - t1).toSec());
            ROS_INFO("[FSM] trajectory desired time: %f", desired_time);
            replan_flag = true;
        }
        else{
            sentryStatus = planningStatus::GEN_NEW_TRAJ;
            ROS_ERROR("[FSM] replan trajectory failed");
            return;
        }
    }

}

void ReplanFSM::checkReplanCallback(const std_msgs::BoolConstPtr &msg)  //检查是否需要重规划
{
    /**
     * @brief 重规划回调标志
     * @param
     */
    if(plannerManager->optimized_path.size() < 2){
        return;
    }

    if(msg->data == true){
        ROS_ERROR("[FSM] replan trajectory now !");
        if(!replan_flag){  /// 防止高频重复replan
            sentryStatus = planningStatus::REPLAN_TRAJ;
        }
    }else{
        replan_flag = false;
        return;
    }
}















