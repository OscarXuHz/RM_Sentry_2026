#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include "decision_node/motion_change.hpp"

class CheckArrived : public BT::ConditionNode
{
public:
  CheckArrived(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bool arrived = bb->get<bool>("nav.arrived");
    // ROS_INFO("CheckArrived: arrived=%d", arrived);
    return arrived ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// CheckAttacked: Checks if the robot is currently under attack by detecting HP decrease above a threshold
class CheckAttacked : public BT::ConditionNode
{
public:
  CheckAttacked(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), last_hp_(-1)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("attack_threshold", 5, "Minimum damage required to trigger attack response"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    int attack_threshold = 5;
    (void)getInput("attack_threshold", attack_threshold);
    
    // Get current HP from blackboard
    try
    {
      int current_hp = bb->get<int>("ref.remain_hp");
      
      // Initialize last_hp on first call
      if (last_hp_ == -1) {
        last_hp_ = current_hp;
        return BT::NodeStatus::FAILURE;  // First frame, not under attack
      }
      
      // Check if HP decreased by at least the threshold (被攻击了)
      int damage_taken = last_hp_ - current_hp;
      bool is_under_attack = (damage_taken >= attack_threshold);
      
      // Update last_hp for next frame
      if (current_hp > last_hp_) {
        // HP increased (healed or respawned), update last_hp
        last_hp_ = current_hp;
      } else if (current_hp < last_hp_) {
        // HP decreased (taking damage)
        last_hp_ = current_hp;
      }
      // If equal, don't update and remember the decreased state
      
      // ROS_DEBUG("CheckAttacked: current_hp=%d, last_hp=%d, damage_taken=%d, threshold=%d, is_under_attack=%d", 
      //           current_hp, last_hp_, damage_taken, attack_threshold, is_under_attack);
      return is_under_attack ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("CheckAttacked: Exception caught - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  int last_hp_;  // Track HP from previous frame
};

class SetMotionFlag : public BT::SyncActionNode
{
public:
  explicit SetMotionFlag(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("target_motion")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    auto target_motion = getInput<int>("target_motion");
    if (!target_motion)
    {
      ROS_WARN("SetMotionFlag: Failed to get target_motion input");
      return BT::NodeStatus::FAILURE;
    }
    
    int target = target_motion.value();
    
    // Get current motion_flag
    int current = 2;  // default value
    try
    {
      current = bb->get<int>("motion_flag");
    }
    catch (...)
    {
      current = 2;
      bb->set("motion_flag", current);
    }

    // Determine if cooldown is needed
    bool needs_cooldown = needsCooldown(current, target);
    
    if (needs_cooldown)
    {
      // Check cooldown time
      ros::Time cooldown_end;
      try
      {
        cooldown_end = bb->get<ros::Time>("attack_cooldown_end_time");
      }
      catch (...)
      {
        cooldown_end = ros::Time(0);
        bb->set("attack_cooldown_end_time", cooldown_end);
      }
      
      bool cooldown_finished = (cooldown_end.toSec() == 0) || (ros::Time::now() >= cooldown_end);
      
      if (!cooldown_finished)
      {
        // Still in cooldown, don't change motion_flag
        ROS_DEBUG("SetMotionFlag: current=%d, target=%d, still in cooldown (%.1fs), skipping", 
                  current, target, (cooldown_end - ros::Time::now()).toSec());
        return BT::NodeStatus::SUCCESS;
      }
      
      // Cooldown finished, set motion_flag and restart cooldown timer
      bb->set("motion_flag", target);
      bb->set("attack_cooldown_end_time", ros::Time::now() + ros::Duration(5.0));
      ROS_DEBUG("SetMotionFlag: Cooldown finished, motion_flag changed from %d to %d, cooldown restarted", 
                current, target);
    }
    else
    {
      // No cooldown needed, set immediately
      bb->set("motion_flag", target);
      // Clear cooldown timer for transitions involving state 3
      bb->set("attack_cooldown_end_time", ros::Time(0));
      ROS_DEBUG("SetMotionFlag: No cooldown needed, motion_flag changed from %d to %d immediately", 
                current, target);
    }
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  // Helper function to determine if cooldown is needed
  // Returns true if: current and target are both in {0,1,2} and different
  // Returns false if: either is 3, or they are the same
  bool needsCooldown(int current, int target)
  {
    // Same state, no cooldown needed
    if (current == target)
      return false;
    
    // If either is state 3 (respawn), no cooldown needed
    if (current == 3 || target == 3)
      return false;
    
    // Both are in {0, 1, 2} and different -> cooldown needed
    return true;
  }
};

// PublishMotion: Publishes motion_flag to ROS topic
class PublishMotion : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  explicit PublishMotion(const std::string& name, const BT::NodeConfiguration& config,
                         ros::Publisher* publisher, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(publisher), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int motion_flag = bb->get<int>("motion_flag");

    if (*publish_on_change_only_)
    {
      // Check if motion_flag has changed since last publish
      int last_motion_flag = -1;
      bool have_last = false;
      try
      {
        last_motion_flag = bb->get<int>("motion.last_flag");
        have_last = true;
      }
      catch (...)
      {
        have_last = false;
      }

      if (have_last && motion_flag == last_motion_flag)
      {
        // ROS_DEBUG("PublishMotion: motion_flag unchanged (%d), skipping publish", motion_flag);
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("motion.last_flag", motion_flag);
    }

    std_msgs::UInt8 msg;
    msg.data = (uint8_t)motion_flag;
    publisher_->publish(msg);
    // ROS_INFO("PublishMotion: published motion_flag = %d", motion_flag);
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterMotionChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* motion_pub, bool* publish_on_change_only)
{
  factory.registerNodeType<CheckArrived>("CheckArrived");
  factory.registerNodeType<CheckAttacked>("CheckAttacked");
  
  factory.registerBuilder<SetMotionFlag>(
      "SetMotionFlag", [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<SetMotionFlag>(name, config);
      });

  factory.registerBuilder<PublishMotion>(
      "PublishMotion", [motion_pub, publish_on_change_only](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<PublishMotion>(name, config, motion_pub, publish_on_change_only);
      });
}

void RegisterPublishMotionNode(BT::BehaviorTreeFactory& factory, ros::Publisher* publisher)
{
  // Note: PublishMotion requires publisher in constructor, needs custom registration
  // This will be handled in strategy_node.cpp
}
