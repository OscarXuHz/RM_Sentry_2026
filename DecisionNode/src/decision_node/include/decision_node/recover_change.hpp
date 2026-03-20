#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

void RegisterRecoverChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* recover_pub, ros::Publisher* bullet_up_pub);

void RegisterBulletSupplyNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* bullet_num_pub);
