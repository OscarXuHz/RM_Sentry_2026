//
// Created by zzt on 23-9-27.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "../include/replan_fsm.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trajectory_generation");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);
    ros::NodeHandle nh("~");

    ReplanFSM rebo_replan;

    rebo_replan.init(nh);

    ros::Duration(4.0).sleep();
    ROS_WARN("start planning");
    rebo_replan.plannerManager->astar_path_finder->visGridMap();


    ros::spin();

    return 0;
}