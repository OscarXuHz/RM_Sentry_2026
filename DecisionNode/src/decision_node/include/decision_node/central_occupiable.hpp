#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

void RegisterOccupationNodes(BT::BehaviorTreeFactory& factory);
void RegisterAccumulateCentralOccupiable(BT::BehaviorTreeFactory& factory);
void RegisterTriggerOnThreshold(BT::BehaviorTreeFactory& factory);
void RegisterResetAccumulator(BT::BehaviorTreeFactory& factory);
void RegisterResetCentralOccupiable(BT::BehaviorTreeFactory& factory);
void RegisterCheckEnemyOccupied(BT::BehaviorTreeFactory& factory);