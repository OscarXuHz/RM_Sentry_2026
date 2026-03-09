#include <ros/ros.h>
#include "../include/tracking_manager.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh("~");

    tracking_manager rebo_replan;
    rebo_replan.init(nh);

    ros::spin();

    return 0;
}