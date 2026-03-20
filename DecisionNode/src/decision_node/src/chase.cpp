#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>

// =====================================================
// InitChase: Initialize chase mode with target selection
// =====================================================
// When entering RADICAL mode:
// 1. Read suggested_target from ref.suggested_target
// 2. Store corresponding enemy position coordinates
// 3. Mark chase as initialized
class InitChase : public BT::SyncActionNode
{
public:
  InitChase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    try
    {
      // Read the suggested target ID from referee data
      uint8_t target_id = bb->get<uint8_t>("ref.suggested_target");
      
      // Get enemy positions from blackboard
      float target_x = 0.0f, target_y = 0.0f;
      
      // Map suggested_target ID to corresponding enemy position
      // suggested_target: 0=Hero, 1=Engineer, 2=Standard-3, 3=Standard-4, 4=Sentry
      switch (target_id)
      {
        case 0:  // Hero
          target_x = bb->get<float>("ref.enemy_hero_x");
          target_y = bb->get<float>("ref.enemy_hero_y");
          ROS_INFO("InitChase: Targeting HERO at (%.2f, %.2f)", target_x, target_y);
          break;

        case 1:  // Engineer
          target_x = bb->get<float>("ref.enemy_engineer_x");
          target_y = bb->get<float>("ref.enemy_engineer_y");
          ROS_INFO("InitChase: Targeting ENGINEER at (%.2f, %.2f)", target_x, target_y);
          break;

        case 2:  // Standard-3
          target_x = bb->get<float>("ref.enemy_standard_3_x");
          target_y = bb->get<float>("ref.enemy_standard_3_y");
          ROS_INFO("InitChase: Targeting STANDARD-3 at (%.2f, %.2f)", target_x, target_y);
          break;

        case 3:  // Standard-4
          target_x = bb->get<float>("ref.enemy_standard_4_x");
          target_y = bb->get<float>("ref.enemy_standard_4_y");
          ROS_INFO("InitChase: Targeting STANDARD-4 at (%.2f, %.2f)", target_x, target_y);
          break;

        case 4:  // Sentry
          target_x = bb->get<float>("ref.enemy_sentry_x");
          target_y = bb->get<float>("ref.enemy_sentry_y");
          ROS_INFO("InitChase: Targeting SENTRY at (%.2f, %.2f)", target_x, target_y);
          break;

        default:
          ROS_WARN("InitChase: Invalid suggested_target ID: %u", target_id);
          return BT::NodeStatus::FAILURE;
      }

      // Store in blackboard
      bb->set("chase.target_id", target_id);
      bb->set("chase.target_x", target_x);
      bb->set("chase.target_y", target_y);
      bb->set("chase.initialized", true);

      ROS_DEBUG("InitChase: Chase initialized with target_id=%u at (%.2f, %.2f)", 
               target_id, target_x, target_y);
      return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("InitChase: Exception caught - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

// =====================================================
// UpdateChaseTarget: Update target position each tick
// =====================================================
// Continuously update the target coordinates in real-time
// This ensures we always have the latest enemy position
class UpdateChaseTarget : public BT::SyncActionNode
{
public:
  UpdateChaseTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    try
    {
      // Check if chase is initialized
      bool initialized = bb->get<bool>("chase.initialized");
      if (!initialized)
      {
        ROS_WARN("UpdateChaseTarget: Chase not initialized yet");
        return BT::NodeStatus::FAILURE;
      }

      // Get current target ID
      uint8_t target_id = bb->get<uint8_t>("chase.target_id");
      
      // Read real-time position based on target ID
      float target_x = 0.0f, target_y = 0.0f;
      
      switch (target_id)
      {
        case 0:  // Hero
          target_x = bb->get<float>("ref.enemy_hero_x");
          target_y = bb->get<float>("ref.enemy_hero_y");
          break;

        case 1:  // Engineer
          target_x = bb->get<float>("ref.enemy_engineer_x");
          target_y = bb->get<float>("ref.enemy_engineer_y");
          break;

        case 2:  // Standard-3
          target_x = bb->get<float>("ref.enemy_standard_3_x");
          target_y = bb->get<float>("ref.enemy_standard_3_y");
          break;

        case 3:  // Standard-4
          target_x = bb->get<float>("ref.enemy_standard_4_x");
          target_y = bb->get<float>("ref.enemy_standard_4_y");
          break;

        case 4:  // Sentry
          target_x = bb->get<float>("ref.enemy_sentry_x");
          target_y = bb->get<float>("ref.enemy_sentry_y");
          break;

        default:
          ROS_WARN("UpdateChaseTarget: Invalid target ID: %u", target_id);
          return BT::NodeStatus::FAILURE;
      }

      // Update position in blackboard
      bb->set("chase.target_x", target_x);
      bb->set("chase.target_y", target_y);

      ROS_DEBUG("UpdateChaseTarget: Target #%u position updated to (%.2f, %.2f)", 
               target_id, target_x, target_y);
      return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("UpdateChaseTarget: Exception caught - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

// =====================================================
// PublishChaseGoal: Publish chase target to clicked_point
// =====================================================
// This node publishes the target position to clicked_point topic
// (same as SetGoalFromParamsCyclic, but with enemy coordinates)
class PublishChaseGoal : public BT::SyncActionNode
{
public:
  explicit PublishChaseGoal(const std::string& name, const BT::NodeConfiguration& config,
                           ros::Publisher* goal_pub, bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), goal_pub_(goal_pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!goal_pub_)
    {
      ROS_WARN("PublishChaseGoal: goal_pub is null");
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;

    try
    {
      // Get current chase target from blackboard
      float target_x = bb->get<float>("chase.target_x");
      float target_y = bb->get<float>("chase.target_y");
      uint8_t target_id = bb->get<uint8_t>("chase.target_id");

      // Create PointStamped message (same format as SetGoalFromParamsCyclic)
      geometry_msgs::PointStamped goal;
      goal.header.frame_id = "map";
      goal.header.stamp = ros::Time::now();
      goal.point.x = target_x;
      goal.point.y = target_y;
      goal.point.z = 0.0;

      // Check if we should publish (based on change_only policy)
      static float last_x = 0.0f, last_y = 0.0f;
      bool should_publish = true;

      if (publish_on_change_only_ && *publish_on_change_only_)
      {
        const float threshold = 0.01f;  // 1cm threshold
        float dx = target_x - last_x;
        float dy = target_y - last_y;
        float distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < threshold)
        {
          should_publish = false;
        }
      }

      if (should_publish)
      {
        goal_pub_->publish(goal);
        last_x = target_x;
        last_y = target_y;
        ROS_DEBUG("PublishChaseGoal: Published goal for target #%u at (%.2f, %.2f) to clicked_point",
                 target_id, target_x, target_y);
      }

      return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("PublishChaseGoal: Exception caught - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  ros::Publisher* goal_pub_;
  bool* publish_on_change_only_;
};

// =====================================================
// ResetChase: Reset chase state when exiting RADICAL
// =====================================================
class ResetChase : public BT::SyncActionNode
{
public:
  ResetChase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    try
    {
      bb->set("chase.target_id", uint8_t(0));
      bb->set("chase.target_x", 0.0f);
      bb->set("chase.target_y", 0.0f);
      bb->set("chase.initialized", false);

      ROS_DEBUG("ResetChase: Chase state reset");
      return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
      ROS_WARN("ResetChase: Exception caught - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

// =====================================================
// Registration function
// =====================================================
void RegisterChaseNodes(BT::BehaviorTreeFactory& factory, ros::Publisher* goal_pub, bool* publish_on_change_only)
{
  factory.registerNodeType<InitChase>("InitChase");
  factory.registerNodeType<UpdateChaseTarget>("UpdateChaseTarget");
  
  factory.registerBuilder<PublishChaseGoal>(
    "PublishChaseGoal", [goal_pub, publish_on_change_only](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishChaseGoal>(name, config, goal_pub, publish_on_change_only);
    });

  factory.registerNodeType<ResetChase>("ResetChase");
}
