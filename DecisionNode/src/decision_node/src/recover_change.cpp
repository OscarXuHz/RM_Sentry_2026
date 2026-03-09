#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <algorithm>
#include "decision_node/recover_change.hpp"

// IsHealthFull: Check if current HP equals max HP
class IsHealthFull : public BT::ConditionNode
{
public:
  IsHealthFull(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("max_hp", 400, "Maximum HP value"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int remain_hp = bb->get<int>("ref.remain_hp");
    
    int max_hp = 400;
    (void)getInput("max_hp", max_hp);
    
    bool is_full = (remain_hp >= max_hp);
    // ROS_INFO("IsHealthFull: remain_hp=%d, max_hp=%d, is_full=%d", remain_hp, max_hp, is_full);
    return is_full ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// SetRecover: Set recover flag to 0 or 1
class SetRecover : public BT::SyncActionNode
{
public:
  SetRecover(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("value", 0, "Recover value: 0 or 1")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int value = 0;
    (void)getInput("value", value);
    
    bb->set("recover", value);
    ROS_DEBUG("SetRecover: recover set to %d", value);
    return BT::NodeStatus::SUCCESS;
  }
};

// IsBulletFull: Check if current bullet equals max bullet
class IsBulletFull : public BT::ConditionNode
{
public:
  IsBulletFull(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("max_bullet", 999, "Maximum bullet value"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int bullet_remain = bb->get<int>("ref.bullet_remain");
    
    int max_bullet = 999;
    (void)getInput("max_bullet", max_bullet);
    
    bool is_full = (bullet_remain >= max_bullet);
    // ROS_INFO("IsBulletFull: bullet_remain=%d, max_bullet=%d, is_full=%d", bullet_remain, max_bullet, is_full);
    return is_full ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// SetBulletUp: Set bullet_up flag to 0 or 1
class SetBulletUp : public BT::SyncActionNode
{
public:
  SetBulletUp(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("value", 0, "BulletUp value: 0 or 1")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int value = 0;
    (void)getInput("value", value);
    
    bb->set("bullet_up", value);
    ROS_DEBUG("SetBulletUp: bullet_up set to %d", value);
    return BT::NodeStatus::SUCCESS;
  }
};

// PublishRecover: Publish recover value to ROS topic
class PublishRecover : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  PublishRecover(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher* pub, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int recover = 0;
    try
    {
      recover = bb->get<int>("recover");
    }
    catch (...)
    {
      recover = 0;
      bb->set("recover", recover);
    }

    if (*publish_on_change_only_)
    {
      int last_recover = 0;
      try
      {
        last_recover = bb->get<int>("recover_last");
      }
      catch (...)
      {
        last_recover = -1;
      }

      if (last_recover == recover)
      {
        return BT::NodeStatus::SUCCESS;  // No change, skip publish
      }
      bb->set("recover_last", recover);
    }

    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(recover);
    ROS_DEBUG("PublishRecover: Publishing recover=%d", recover);
    publisher_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }
};

// PublishBulletUp: Publish bullet_up value to ROS topic
class PublishBulletUp : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  PublishBulletUp(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher* pub, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int bullet_up = 0;
    try
    {
      bullet_up = bb->get<int>("bullet_up");
    }
    catch (...)
    {
      bullet_up = 0;
      bb->set("bullet_up", bullet_up);
    }

    if (*publish_on_change_only_)
    {
      int last_bullet_up = 0;
      try
      {
        last_bullet_up = bb->get<int>("bullet_up_last");
      }
      catch (...)
      {
        last_bullet_up = -1;
      }

      if (last_bullet_up == bullet_up)
      {
        return BT::NodeStatus::SUCCESS;  // No change, skip publish
      }
      bb->set("bullet_up_last", bullet_up);
    }

    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(bullet_up);
    ROS_DEBUG("PublishBulletUp: Publishing bullet_up=%d", bullet_up);
    publisher_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }
};

// SetBulletNum: Set bullet_num based on strategy (delta mode or fixed mode)
// Mode 1: DELTA - Calculate difference between expected and current
// Mode 2: FIXED - Use fixed supply amount
class SetBulletNum : public BT::SyncActionNode
{
public:
  SetBulletNum(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "DELTA", "Supply mode: DELTA (expected-current) or FIXED (fixed amount)"),
      BT::InputPort<int>("expected_bullet", 999, "Expected maximum bullet count (for DELTA mode)"),
      BT::InputPort<int>("fixed_supply", 50, "Fixed supply amount (for FIXED mode)"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    
    std::string mode = "DELTA";
    int expected_bullet = 999;
    int fixed_supply = 50;
    
    (void)getInput("mode", mode);
    (void)getInput("expected_bullet", expected_bullet);
    (void)getInput("fixed_supply", fixed_supply);
    
    // Transform mode to uppercase for case-insensitive comparison
    std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
    
    int current_bullet = 0;
    try
    {
      current_bullet = bb->get<int>("ref.bullet_remain");
    }
    catch (...)
    {
      ROS_WARN("SetBulletNum: Failed to get current bullet_remain from blackboard");
      current_bullet = 0;
    }
    
    int bullet_num = 0;
    
    if (mode == "DELTA")
    {
      // Delta mode: Calculate difference
      bullet_num = std::max(0, expected_bullet - current_bullet);
      ROS_DEBUG("SetBulletNum (DELTA): expected=%d, current=%d, bullet_num=%d", 
                expected_bullet, current_bullet, bullet_num);
    }
    else if (mode == "FIXED")
    {
      // Fixed mode: Use fixed supply amount
      bullet_num = fixed_supply;
      ROS_DEBUG("SetBulletNum (FIXED): bullet_num=%d", bullet_num);
    }
    else
    {
      ROS_WARN("SetBulletNum: Unknown mode '%s', defaulting to DELTA", mode.c_str());
      bullet_num = std::max(0, expected_bullet - current_bullet);
    }
    
    bb->set("bullet_num", bullet_num);
    return BT::NodeStatus::SUCCESS;
  }
};

// PublishBulletNum: Publish bullet_num value to ROS topic
class PublishBulletNum : public BT::SyncActionNode
{
private:
  ros::Publisher* publisher_;
  bool* publish_on_change_only_;

public:
  PublishBulletNum(const std::string& name, const BT::NodeConfiguration& config, ros::Publisher* pub, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), publisher_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int bullet_num = 0;
    try
    {
      bullet_num = bb->get<int>("bullet_num");
    }
    catch (...)
    {
      bullet_num = 0;
      bb->set("bullet_num", bullet_num);
    }

    if (*publish_on_change_only_)
    {
      int last_bullet_num = 0;
      try
      {
        last_bullet_num = bb->get<int>("bullet_num_last");
      }
      catch (...)
      {
        last_bullet_num = -1;
      }

      if (last_bullet_num == bullet_num)
      {
        return BT::NodeStatus::SUCCESS;  // No change, skip publish
      }
      bb->set("bullet_num_last", bullet_num);
    }

    std_msgs::UInt8 msg;
    msg.data = static_cast<uint8_t>(bullet_num);
    ROS_DEBUG("PublishBulletNum: Publishing bullet_num=%d", bullet_num);
    publisher_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }
};

void RegisterRecoverChangeNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* recover_pub, ros::Publisher* bullet_up_pub)
{
  factory.registerNodeType<IsHealthFull>("IsHealthFull");
  factory.registerNodeType<SetRecover>("SetRecover");
  factory.registerNodeType<IsBulletFull>("IsBulletFull");
  factory.registerNodeType<SetBulletUp>("SetBulletUp");
  
  // recover_pub 为 nullptr 时不注册 PublishRecover（可选）
  if (recover_pub != nullptr)
  {
    factory.registerBuilder<PublishRecover>(
      "PublishRecover", [recover_pub](const std::string& name, const BT::NodeConfiguration& config) {
        static bool publish_on_change = true;  // 可根据需要调整
        return std::make_unique<PublishRecover>(name, config, recover_pub, &publish_on_change);
      });
  }
  
  // bullet_up_pub 为 nullptr 时不注册 PublishBulletUp（可选）
  if (bullet_up_pub != nullptr)
  {
    factory.registerBuilder<PublishBulletUp>(
      "PublishBulletUp", [bullet_up_pub](const std::string& name, const BT::NodeConfiguration& config) {
        static bool publish_on_change = true;  // 可根据需要调整
        return std::make_unique<PublishBulletUp>(name, config, bullet_up_pub, &publish_on_change);
      });
  }
}

void RegisterBulletSupplyNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* bullet_num_pub)
{
  factory.registerNodeType<SetBulletNum>("SetBulletNum");
  
  if (bullet_num_pub != nullptr)
  {
    factory.registerBuilder<PublishBulletNum>(
      "PublishBulletNum", [bullet_num_pub](const std::string& name, const BT::NodeConfiguration& config) {
        static bool publish_on_change = true;  // 可根据需要调整
        return std::make_unique<PublishBulletNum>(name, config, bullet_num_pub, &publish_on_change);
      });
  }
}
