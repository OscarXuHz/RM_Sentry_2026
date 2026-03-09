#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

void RegisterMotionChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* motion_pub, bool* publish_on_change_only);
