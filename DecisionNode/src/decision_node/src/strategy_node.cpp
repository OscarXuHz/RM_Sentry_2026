/*
决策模块
通信：
  订阅裁判系统相关话题（占位，后续可替换为 Referee_Task 的真实桥接）
  订阅导航到达状态话题（复用现有语义）
  发布目标点话题（clicked_point）
  发布烧饼状态话题（motion）
[TODO] 
- 后续可接真实视觉数据
- 修改上下位机通讯逻辑，只负责运动控制的收发
- 在决策判断是否到达感觉很别扭，是否放在上下位机通讯模块更合适？
*/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/decorators/inverter_node.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

#include "decision_node/central_occupiable.hpp"
#include "decision_node/motion_change.hpp"
#include "decision_node/recover_change.hpp"
#include "decision_node/chase.hpp"
namespace
{
constexpr int kDefaultTickHz = 20;

std::string toUpper(std::string s)
{
  for (auto& c : s)
  {
    c = static_cast<char>(::toupper(c));
  }
  return s;
}

}  // namespace

// ---------------------------
// Shared state (ROS callbacks)
// ---------------------------
struct RefereeState
{
  int game_progress = 0;  // 0，1，2，3: not start, 4: in progress, 5: end (convention)
  int remain_hp = 400;
  int bullet_remain = 999;
  int friendly_score = 0;
  int enemy_score = 0;
  int occupy_status = 0;  // 0: unoccupied, 1: friendly occupied, 2: enemy occupied，3：both occupied
  int robot_id = 0;       // 机器人ID
  int robot_color = 0;    // 机器人颜色 (0=red, 1=blue)
  int self_hp = 400;      // 自身血量
  int self_max_hp = 400;  // 自身最大血量
  int red_1_hp = 400;     // 红英雄血量
  int red_3_hp = 400;     // 红步兵3血量
  int red_7_hp = 400;     // 红哨兵血量
  int blue_1_hp = 400;    // 蓝英雄血量
  int blue_3_hp = 400;    // 蓝步兵3血量
  int blue_7_hp = 400;    // 蓝哨兵血量
  int red_dead = 0;       // 红方死亡位
  int blue_dead = 0;      // 蓝方死亡位
  // Enemy positions for chase mode
  float enemy_hero_x = 0.0f;
  float enemy_hero_y = 0.0f;
  float enemy_engineer_x = 0.0f;
  float enemy_engineer_y = 0.0f;
  float enemy_standard_3_x = 0.0f;
  float enemy_standard_3_y = 0.0f;
  float enemy_standard_4_x = 0.0f;
  float enemy_standard_4_y = 0.0f;
  float enemy_sentry_x = 0.0f;
  float enemy_sentry_y = 0.0f;
  uint8_t suggested_target = 0;  // Target suggestion from radar
};

struct NavigationState
{
  bool arrived = false;
};

// ---------------------------
// BT Nodes: Update blackboard
// ---------------------------
class UpdateRefereeBB : public BT::SyncActionNode
{
public:
  UpdateRefereeBB(const std::string& name, const BT::NodeConfiguration& config, const RefereeState* state)
    : BT::SyncActionNode(name, config), state_(state),
      cached_enemy_hero_x_(0.0f), cached_enemy_hero_y_(0.0f),
      cached_enemy_engineer_x_(0.0f), cached_enemy_engineer_y_(0.0f),
      cached_enemy_standard_3_x_(0.0f), cached_enemy_standard_3_y_(0.0f),
      cached_enemy_standard_4_x_(0.0f), cached_enemy_standard_4_y_(0.0f),
      cached_enemy_sentry_x_(0.0f), cached_enemy_sentry_y_(0.0f)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    bb->set("ref.game_progress", state_->game_progress);
    bb->set("ref.remain_hp", state_->remain_hp);
    bb->set("ref.bullet_remain", state_->bullet_remain);
    bb->set("ref.friendly_score", state_->friendly_score);
    bb->set("ref.enemy_score", state_->enemy_score);
    bb->set("ref.occupy_status", state_->occupy_status);
    bb->set("ref.robot_id", state_->robot_id);
    bb->set("ref.robot_color", state_->robot_color);
    bb->set("ref.self_hp", state_->self_hp);
    bb->set("ref.self_max_hp", state_->self_max_hp);
    bb->set("ref.red_1_hp", state_->red_1_hp);
    bb->set("ref.red_3_hp", state_->red_3_hp);
    bb->set("ref.red_7_hp", state_->red_7_hp);
    bb->set("ref.blue_1_hp", state_->blue_1_hp);
    bb->set("ref.blue_3_hp", state_->blue_3_hp);
    bb->set("ref.blue_7_hp", state_->blue_7_hp);
    bb->set("ref.red_dead", state_->red_dead);
    bb->set("ref.blue_dead", state_->blue_dead);
    
    // Enemy positions - 检测-8888无效值，只在有效时更新缓存
    if (state_->enemy_hero_x != -8888.0f) cached_enemy_hero_x_ = state_->enemy_hero_x;
    if (state_->enemy_hero_y != -8888.0f) cached_enemy_hero_y_ = state_->enemy_hero_y;
    if (state_->enemy_engineer_x != -8888.0f) cached_enemy_engineer_x_ = state_->enemy_engineer_x;
    if (state_->enemy_engineer_y != -8888.0f) cached_enemy_engineer_y_ = state_->enemy_engineer_y;
    if (state_->enemy_standard_3_x != -8888.0f) cached_enemy_standard_3_x_ = state_->enemy_standard_3_x;
    if (state_->enemy_standard_3_y != -8888.0f) cached_enemy_standard_3_y_ = state_->enemy_standard_3_y;
    if (state_->enemy_standard_4_x != -8888.0f) cached_enemy_standard_4_x_ = state_->enemy_standard_4_x;
    if (state_->enemy_standard_4_y != -8888.0f) cached_enemy_standard_4_y_ = state_->enemy_standard_4_y;
    if (state_->enemy_sentry_x != -8888.0f) cached_enemy_sentry_x_ = state_->enemy_sentry_x;
    if (state_->enemy_sentry_y != -8888.0f) cached_enemy_sentry_y_ = state_->enemy_sentry_y;
    
    // 发布缓存中的敌方位置到blackboard
    bb->set("ref.enemy_hero_x", cached_enemy_hero_x_);
    bb->set("ref.enemy_hero_y", cached_enemy_hero_y_);
    bb->set("ref.enemy_engineer_x", cached_enemy_engineer_x_);
    bb->set("ref.enemy_engineer_y", cached_enemy_engineer_y_);
    bb->set("ref.enemy_standard_3_x", cached_enemy_standard_3_x_);
    bb->set("ref.enemy_standard_3_y", cached_enemy_standard_3_y_);
    bb->set("ref.enemy_standard_4_x", cached_enemy_standard_4_x_);
    bb->set("ref.enemy_standard_4_y", cached_enemy_standard_4_y_);
    bb->set("ref.enemy_sentry_x", cached_enemy_sentry_x_);
    bb->set("ref.enemy_sentry_y", cached_enemy_sentry_y_);
    bb->set("ref.suggested_target", state_->suggested_target);
    return BT::NodeStatus::SUCCESS;
  }

private:
  const RefereeState* state_;
  // 缓存敌方位置 - 用于处理-8888无效值
  float cached_enemy_hero_x_;
  float cached_enemy_hero_y_;
  float cached_enemy_engineer_x_;
  float cached_enemy_engineer_y_;
  float cached_enemy_standard_3_x_;
  float cached_enemy_standard_3_y_;
  float cached_enemy_standard_4_x_;
  float cached_enemy_standard_4_y_;
  float cached_enemy_sentry_x_;
  float cached_enemy_sentry_y_;
};

class UpdateNavigationBB : public BT::SyncActionNode
{
public:
  UpdateNavigationBB(const std::string& name, const BT::NodeConfiguration& config, const NavigationState* state)
    : BT::SyncActionNode(name, config), state_(state)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    config().blackboard->set("nav.arrived", state_->arrived);
    return BT::NodeStatus::SUCCESS;
  }

private:
  const NavigationState* state_;
};

class UpdateVisionBB : public BT::SyncActionNode
{
public:
  UpdateVisionBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // V1: explicitly弃用视觉模块。这里保持节点存在，但不写入任何视觉字段。[TODO] 后续可接真实视觉数据
    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateTimersBB : public BT::SyncActionNode
{
public:
  UpdateTimersBB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // 预留：[TODO] 后续可把 Strategy_Task.c 里的计数器/超时机制迁移到这里
    return BT::NodeStatus::SUCCESS;
  }
};

class UpdateDerivedFlags : public BT::SyncActionNode
{
public:
  UpdateDerivedFlags(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("danger_hp", 100, "HP threshold: below which is considered 'in danger'"),
      BT::InputPort<int>("sufficient_bullet", 10, "Bullet threshold: below which is considered insufficient"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;

    const int remain_hp = bb->get<int>("ref.remain_hp");
    const int bullet_remain = bb->get<int>("ref.bullet_remain");
    
    // --- Damage Tracking Logic ---
    ros::Time now = ros::Time::now();
    
    // Initialize last_hp_ on first run or if it was reset
    if (last_hp_ == -1) {
        last_hp_ = remain_hp;
    }

    // Detect damage (hp drop)
    // Only accumulate damage if we are consistently tracking. 
    // If massive jump up (respawn?), reset? For now just track drops.
    if (remain_hp < last_hp_) {
        int damage = last_hp_ - remain_hp;
        damage_history_.push_back({now, damage});
        // ROS_DEBUG("UpdateDerivedFlags: Detected damage %d. History size: %lu", damage, damage_history_.size());
    } else if (remain_hp > last_hp_) {
        // Healed or respawned
        // Do we reset history on respawn? Maybe not necessary for small heals.
        // If respawn (hp jump to max), maybe clear history? 
        // Assuming standard healing, we just update last_hp_.
    }
    last_hp_ = remain_hp;

    // Prune history older than 2 seconds
    while (!damage_history_.empty()) {
        double time_diff = (now - damage_history_.front().first).toSec();
        if (time_diff > 2.0) {
            damage_history_.pop_front();
        } else {
            break; 
        }
    }

    // Sum damage in window
    int total_damage_2s = 0;
    for (const auto& entry : damage_history_) {
        total_damage_2s += entry.second;
    }
    
    bb->set("derived.damage_2s", total_damage_2s);
    // ----------------------------

    // ROS_INFO("UpdateDerivedFlags: remain_hp=%d, bullet_remain=%d", remain_hp, bullet_remain);

    int danger_hp = 100;
    (void)getInput("danger_hp", danger_hp);

    int sufficient_bullet = 10;
    (void)getInput("sufficient_bullet", sufficient_bullet);

    const bool is_dead = (remain_hp <= 0);
    const bool is_in_danger = (remain_hp > 0 && remain_hp < danger_hp);
    const bool bullet_sufficient = (bullet_remain >= sufficient_bullet);

    bb->set("is_dead", is_dead);
    bb->set("is_in_danger", is_in_danger);
    bb->set("bullet_sufficient", bullet_sufficient);

    // V1: central_occupiable 先作为占位，后续接裁判/受击统计
    try
    {
      (void)bb->get<bool>("central_occupiable");
    }
    catch (...)
    {
      bb->set("central_occupiable", false);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  int last_hp_ = -1;
  std::deque<std::pair<ros::Time, int>> damage_history_;
};

class IntenseHarm : public BT::ConditionNode
{
public:
  IntenseHarm(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), is_active_(false)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("threshold_activate", 100, "Damage threshold to activate"),
      BT::InputPort<int>("threshold_deactivate", 50, "Damage threshold to deactivate"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int damage_2s = 0;
    try {
        damage_2s = bb->get<int>("derived.damage_2s");
    } catch (...) {
        damage_2s = 0;
    }

    int t_on = 100;
    int t_off = 50;
    getInput("threshold_activate", t_on);
    getInput("threshold_deactivate", t_off);

    if (!is_active_) {
        if (damage_2s > t_on) {
            is_active_ = true;
            ROS_INFO("IntenseHarm: Activated! Damage(2s)=%d >= %d", damage_2s, t_on);
        }
    } else {
        if (damage_2s < t_off) {
            is_active_ = false;
            ROS_INFO("IntenseHarm: Deactivated! Damage(2s)=%d < %d", damage_2s, t_off);
        }
    }

    return is_active_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  bool is_active_;
};

// ---------------------------
// BT Nodes: Conditions
// ---------------------------
class IsGameStarted : public BT::ConditionNode
{
public:
  IsGameStarted(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() 
  { 
    return {BT::InputPort<bool>("expect_started", true, "Expected game state: true=started, false=not started")};
  }

  BT::NodeStatus tick() override
  {
    const int gp = config().blackboard->get<int>("ref.game_progress");
    const bool is_started = (gp == 4);  // 4 means game in progress
    
    // Get the expect_started parameter, default to true if not provided
    const bool expect_started = getInput<bool>("expect_started").value_or(true);
    
    // Return SUCCESS if actual state matches expected state
    const bool condition_met = (is_started == expect_started);
    return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryDead : public BT::ConditionNode
{
public:
  IsSentryDead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool is_dead = config().blackboard->get<bool>("is_dead");
    return is_dead ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryAlive : public BT::ConditionNode
{
public:
  IsSentryAlive(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool is_dead = config().blackboard->get<bool>("is_dead");
    return (!is_dead) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsSentryInDanger : public BT::ConditionNode
{
public:
  IsSentryInDanger(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool in_danger = config().blackboard->get<bool>("is_in_danger");
    return in_danger ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class NotBulletSufficient : public BT::ConditionNode
{
public:
  NotBulletSufficient(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const bool bullet_sufficient = config().blackboard->get<bool>("bullet_sufficient");
    return (!bullet_sufficient) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class AggressiveAdvantage : public BT::ConditionNode
{
public:
  AggressiveAdvantage(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("threshold", 50, "Score advantage threshold to be aggressive"),
    };
  }

  BT::NodeStatus tick() override
  {
    const int friendly_score = config().blackboard->get<int>("ref.friendly_score");
    const int enemy_score = config().blackboard->get<int>("ref.enemy_score");

    int threshold = 50;
    (void)getInput("threshold", threshold);

    const int score_advantage = friendly_score - enemy_score;

    // Add logging for debugging
    // ROS_INFO_STREAM("[AggressiveAdvantage] Friendly Score: " << friendly_score);
    // ROS_INFO_STREAM("[AggressiveAdvantage] Enemy Score: " << enemy_score);
    // ROS_INFO_STREAM("[AggressiveAdvantage] Score Advantage: " << score_advantage);
    // ROS_INFO_STREAM("[AggressiveAdvantage] Threshold: " << threshold);

    return (score_advantage >= threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};



class IsAction : public BT::ConditionNode
{
public:
  IsAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("value", "INIT", "Action name")};
  }

  BT::NodeStatus tick() override
  {
    std::string value;
    if (!getInput("value", value))
    {
      return BT::NodeStatus::FAILURE;
    }

    std::string action;
    try
    {
      action = config().blackboard->get<std::string>("action");
    }
    catch (...)
    {
      return BT::NodeStatus::FAILURE;
    }

    bool result = (toUpper(action) == toUpper(value));
    // ROS_INFO("IsAction: checking action=%s, expect=%s, result=%d", 
    //          toUpper(action).c_str(), toUpper(value).c_str(), result);
    return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ---------------------------
// BT Nodes: Actions
// ---------------------------
class SetAction : public BT::SyncActionNode
{
public:
  SetAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action")}; }

  BT::NodeStatus tick() override
  {
    std::string action;
    if (!getInput("action", action))
    {
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set("action", toUpper(action));
    // ROS_INFO("SetAction: action set to %s", toUpper(action).c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class ClearGoal : public BT::SyncActionNode
{
public:
  ClearGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    // ROS_INFO("ClearGoal: setting goal.valid = false");
    bb->set("goal.valid", false);
    return BT::NodeStatus::SUCCESS;
  }
};

// Wait: 等待指定的秒数
class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), wait_end_time_(ros::Time(0))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 1.0, "等待的秒数"),
    };
  }

  BT::NodeStatus tick() override
  {
    double duration = 1.0;
    if (!getInput("duration", duration))
    {
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    
    // 第一次进入时，记录开始时间
    if (wait_end_time_ == ros::Time(0))
    {
      wait_end_time_ = ros::Time::now() + ros::Duration(duration);
      ROS_DEBUG("Wait: Starting wait for %.2f seconds", duration);
      return BT::NodeStatus::RUNNING;
    }

    // 检查是否超过等待时间
    if (ros::Time::now() >= wait_end_time_)
    {
      ROS_DEBUG("Wait: Wait completed");
      wait_end_time_ = ros::Time(0);  // 重置计时器
      return BT::NodeStatus::SUCCESS;
    }

    // 还没等够，继续等待
    return BT::NodeStatus::RUNNING;
  }

private:
  ros::Time wait_end_time_;
};

//[TODO]: 这里的目标点是从参数服务器读取的，
class SetGoalFromParams : public BT::SyncActionNode
{
public:
  SetGoalFromParams(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "push", "parameter namespace: e.g. push/occupy/supply"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string ns;
    if (!getInput("ns", ns))
    {
      ROS_WARN("SetGoalFromParams: Failed to get 'ns' input");
      return BT::NodeStatus::FAILURE;
    }

    const std::string base = std::string("goals/") + ns;

    double x = 0.0, y = 0.0;
    (void)nh_->param(base + "/x", x, 0.0);
    (void)nh_->param(base + "/y", y, 0.0);

    ROS_DEBUG("SetGoalFromParams: ns=%s, goal=(%f, %f)", ns.c_str(), x, y);

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    auto bb = config().blackboard;
    bb->set("goal.point", goal);
    bb->set("goal.valid", true);  
    ROS_DEBUG("SetGoalFromParams: goal.valid set to TRUE");
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
};

class SetGoalFromParamsCyclic : public BT::SyncActionNode
{
public:
  SetGoalFromParamsCyclic(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle* nh)
    : BT::SyncActionNode(name, config), nh_(nh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ns", "push", "parameter namespace: e.g. push/occupy"),
      BT::InputPort<int>("point_count", 4, "number of points to cycle through"),
    };
  }

  BT::NodeStatus tick() override
  {
    // ROS_INFO("SetGoalFromParamsCyclic: ENTERED");
    std::string ns;
    int point_count = 4;
    if (!getInput("ns", ns))
    {
      ROS_WARN("SetGoalFromParamsCyclic: Failed to get 'ns' input");
      return BT::NodeStatus::FAILURE;
    }
    // ROS_INFO("SetGoalFromParamsCyclic: ns=%s", ns.c_str());
    (void)getInput("point_count", point_count);

    auto bb = config().blackboard;
    
    // Get current cycle index
    int cycle_index = 0;
    try {
      cycle_index = bb->get<int>("goal.cycle_index");
    } catch (...) {
      cycle_index = 0;
      bb->set("goal.cycle_index", cycle_index);
    }

    // Read goal from parameters: goals/<ns>/point_<index>/{x,y}
    const std::string base = std::string("goals/") + ns + "/point_" + std::to_string(cycle_index);

    double x = 0.0, y = 0.0;
    (void)nh_->param(base + "/x", x, 0.0);
    (void)nh_->param(base + "/y", y, 0.0);

    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = x;
    goal.point.y = y;
    goal.point.z = 0.0;

    bb->set("goal.point", goal);
    bb->set("goal.valid", true);  
    // ROS_INFO("SetGoalFromParamsCyclic: ns=%s, point_count=%d, cycle_index=%d, goal=(%f, %f), goal.valid set to TRUE", 
    //          ns.c_str(), point_count, cycle_index, x, y);
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle* nh_;
};

class AdvanceCycleIndex : public BT::SyncActionNode
{
public:
  AdvanceCycleIndex(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("point_count", 4, "number of points to cycle through"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    int point_count = 4;
    (void)getInput("point_count", point_count);

    int cycle_index = 0;
    try {
      cycle_index = bb->get<int>("goal.cycle_index");
    } catch (...) {
      cycle_index = 0;
    }

    // Advance to next point and wrap around
    cycle_index = (cycle_index + 1) % point_count;
    bb->set("goal.cycle_index", cycle_index);
    // ROS_INFO("AdvanceCycleIndex: Advanced to cycle_index=%d", cycle_index);

    return BT::NodeStatus::SUCCESS;
  }
};

class PublishGoalPoint : public BT::SyncActionNode
{
public:
  PublishGoalPoint(const std::string& name,
                   const BT::NodeConfiguration& config,
                   ros::Publisher* pub,
                   bool* publish_on_change_only)
    : BT::SyncActionNode(name, config), pub_(pub), publish_on_change_only_(publish_on_change_only)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "clicked_point", "target topic"),
    };
  }

  BT::NodeStatus tick() override
  {
    (void)getInput("topic", last_topic_);  // retained only for XML readability

    auto bb = config().blackboard;
    const bool valid = bb->get<bool>("goal.valid");
    // ROS_INFO("PublishGoalPoint: goal.valid=%d", valid);
    if (!valid)
    {
      // ROS_INFO("PublishGoalPoint: goal.valid is FALSE, skipping publish");
      return BT::NodeStatus::SUCCESS;
    }

    const auto goal = bb->get<geometry_msgs::PointStamped>("goal.point");
    // ROS_INFO("PublishGoalPoint: goal position=(%f, %f)", goal.point.x, goal.point.y);

    if (*publish_on_change_only_)
    {
      // crude de-dup: compare x/y only
      bool have_last = false;
      double last_x = 0.0, last_y = 0.0;
      try
      {
        last_x = bb->get<double>("goal.last_x");
        last_y = bb->get<double>("goal.last_y");
        have_last = true;
      }
      catch (...)
      {
        have_last = false;
      }

      if (have_last && goal.point.x == last_x && goal.point.y == last_y)
      {
        // ROS_INFO("PublishGoalPoint: Position unchanged, skipping publish");
        return BT::NodeStatus::SUCCESS;
      }
      bb->set("goal.last_x", goal.point.x);
      bb->set("goal.last_y", goal.point.y);
    }

    // ROS_INFO("PublishGoalPoint: Publishing goal at (%f, %f)", goal.point.x, goal.point.y);
    pub_->publish(goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::Publisher* pub_;
  bool* publish_on_change_only_;
  std::string last_topic_;
};

// ---------------------------
// Main
// ---------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "strategy_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RefereeState ref;
  NavigationState nav;

  auto blackboard = BT::Blackboard::create();
  
  auto sub_game_progress = nh.subscribe<std_msgs::UInt8>("/referee/game_progress", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.game_progress = msg->data;
  });
  auto sub_remain_hp = nh.subscribe<std_msgs::UInt16>("/referee/remain_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.remain_hp = msg->data;
  });
  auto sub_bullet = nh.subscribe<std_msgs::UInt16>("/referee/bullet_remain", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.bullet_remain = msg->data;
  });
  auto sub_friendly_score = nh.subscribe<std_msgs::Int32>("/referee/friendly_score", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.friendly_score = msg->data;
  });
  auto sub_enemy_score = nh.subscribe<std_msgs::Int32>("/referee/enemy_score", 1, [&](const std_msgs::Int32::ConstPtr& msg) {
    ref.enemy_score = msg->data;
  });
  auto sub_occupy_status = nh.subscribe<std_msgs::UInt8>("/referee/occupy_status", 1,[&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.occupy_status = msg->data;
  });
  auto sub_robot_id = nh.subscribe<std_msgs::UInt8>("/robot/robot_id", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.robot_id = msg->data;
  });
  auto sub_robot_color = nh.subscribe<std_msgs::UInt8>("/robot/robot_color", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.robot_color = msg->data;
  });
  auto sub_self_hp = nh.subscribe<std_msgs::UInt16>("/robot/self_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.self_hp = msg->data;
  });
  auto sub_self_max_hp = nh.subscribe<std_msgs::UInt16>("/robot/self_max_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.self_max_hp = msg->data;
  });
  auto sub_red_1_hp = nh.subscribe<std_msgs::UInt16>("/referee/red_1_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.red_1_hp = msg->data;
  });
  auto sub_red_3_hp = nh.subscribe<std_msgs::UInt16>("/referee/red_3_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.red_3_hp = msg->data;
  });
  auto sub_red_7_hp = nh.subscribe<std_msgs::UInt16>("/referee/red_7_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.red_7_hp = msg->data;
  });
  auto sub_blue_1_hp = nh.subscribe<std_msgs::UInt16>("/referee/blue_1_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.blue_1_hp = msg->data;
  });
  auto sub_blue_3_hp = nh.subscribe<std_msgs::UInt16>("/referee/blue_3_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.blue_3_hp = msg->data;
  });
  auto sub_blue_7_hp = nh.subscribe<std_msgs::UInt16>("/referee/blue_7_hp", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.blue_7_hp = msg->data;
  });
  auto sub_red_dead = nh.subscribe<std_msgs::UInt16>("/referee/red_dead", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.red_dead = msg->data;
  });
  auto sub_blue_dead = nh.subscribe<std_msgs::UInt16>("/referee/blue_dead", 1, [&](const std_msgs::UInt16::ConstPtr& msg) {
    ref.blue_dead = msg->data;
  });
  
  // Enemy positions (for chase mode)
  auto sub_enemy_hero = nh.subscribe<geometry_msgs::Point>("/enemy/hero_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_hero_x = msg->x;
    ref.enemy_hero_y = msg->y;
  });
  auto sub_enemy_engineer = nh.subscribe<geometry_msgs::Point>("/enemy/engineer_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_engineer_x = msg->x;
    ref.enemy_engineer_y = msg->y;
  });
  auto sub_enemy_standard_3 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_3_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_standard_3_x = msg->x;
    ref.enemy_standard_3_y = msg->y;
  });
  auto sub_enemy_standard_4 = nh.subscribe<geometry_msgs::Point>("/enemy/standard_4_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_standard_4_x = msg->x;
    ref.enemy_standard_4_y = msg->y;
  });
  auto sub_enemy_sentry = nh.subscribe<geometry_msgs::Point>("/enemy/sentry_position", 1, [&](const geometry_msgs::Point::ConstPtr& msg) {
    ref.enemy_sentry_x = msg->x;
    ref.enemy_sentry_y = msg->y;
  });
  auto sub_suggested_target = nh.subscribe<std_msgs::UInt8>("/radar/suggested_target", 1, [&](const std_msgs::UInt8::ConstPtr& msg) {
    ref.suggested_target = msg->data;
  });
  
  // Navigation arrived (复用现有语义)
  auto sub_arrived = nh.subscribe<std_msgs::Bool>("/dstar_status", 1, [&](const std_msgs::Bool::ConstPtr& msg) {
    nav.arrived = msg->data;
  });

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1);
  ros::Publisher motion_pub = nh.advertise<std_msgs::UInt8>("motion", 1);
  ros::Publisher recover_pub = nh.advertise<std_msgs::UInt8>("recover", 1);
  ros::Publisher bullet_up_pub = nh.advertise<std_msgs::UInt8>("bullet_up", 1);
  ros::Publisher bullet_num_pub = nh.advertise<std_msgs::UInt8>("bullet_num", 1);

  int tick_hz = kDefaultTickHz;
  pnh.param("tick_hz", tick_hz, tick_hz);

  bool publish_on_change_only = true;
  pnh.param("publish_on_change_only", publish_on_change_only, publish_on_change_only);

  BT::BehaviorTreeFactory factory;

  // Register custom nodes.
  factory.registerBuilder<UpdateRefereeBB>(
    "UpdateRefereeBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateRefereeBB>(name, config, &ref);
    });

  factory.registerBuilder<UpdateNavigationBB>(
    "UpdateNavigationBB", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<UpdateNavigationBB>(name, config, &nav);
    });

  factory.registerNodeType<UpdateVisionBB>("UpdateVisionBB");
  factory.registerNodeType<UpdateTimersBB>("UpdateTimersBB");
  factory.registerNodeType<UpdateDerivedFlags>("UpdateDerivedFlags");

  factory.registerNodeType<IsGameStarted>("IsGameStarted");
  factory.registerNodeType<IsSentryDead>("IsSentryDead");
  factory.registerNodeType<IsSentryAlive>("IsSentryAlive");
  factory.registerNodeType<IsSentryInDanger>("IsSentryInDanger");
  factory.registerNodeType<IntenseHarm>("IntenseHarm");
  factory.registerNodeType<NotBulletSufficient>("NotBulletSufficient");
  factory.registerNodeType<AggressiveAdvantage>("AggressiveAdvantage");
  factory.registerNodeType<IsAction>("IsAction");

  factory.registerNodeType<SetAction>("SetAction");
  factory.registerNodeType<ClearGoal>("ClearGoal");
  factory.registerNodeType<Wait>("Wait");
  
  RegisterMotionChangeNodes(factory, &motion_pub, &publish_on_change_only);

  factory.registerBuilder<SetGoalFromParams>(
    "SetGoalFromParams", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParams>(name, config, &nh);
    });

  factory.registerBuilder<SetGoalFromParamsCyclic>(
    "SetGoalFromParamsCyclic", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<SetGoalFromParamsCyclic>(name, config, &nh);
    });

  factory.registerNodeType<AdvanceCycleIndex>("AdvanceCycleIndex");

  factory.registerBuilder<PublishGoalPoint>(
    "PublishGoalPoint", [&](const std::string& name, const BT::NodeConfiguration& config) {
      return std::make_unique<PublishGoalPoint>(name, config, &goal_pub, &publish_on_change_only);
    });

  RegisterAccumulateCentralOccupiable(factory);
  RegisterResetCentralOccupiable(factory);
  RegisterOccupationNodes(factory);

  RegisterRecoverChangeNodes(factory, &recover_pub, &bullet_up_pub);
  RegisterBulletSupplyNodes(factory, &bullet_num_pub);
  RegisterChaseNodes(factory, &goal_pub, &publish_on_change_only);

  // ---------------------------
  // Decision Parameters (集中定义)
  // ---------------------------
  struct DecisionParams {
    int danger_hp = 100;
    int sufficient_bullet = 10;
    int max_bullet = 150;
    int fixed_supply = 50;
    int occupy_threshold = 30;
    int aggressive_threshold = 50;
    int attack_threshold = 5;
    int harm_threshold_on = 50;
    int harm_threshold_off = 10;
  } params;


  pnh.param("danger_hp", params.danger_hp, params.danger_hp);
  pnh.param("sufficient_bullet", params.sufficient_bullet, params.sufficient_bullet);
  pnh.param("max_bullet", params.max_bullet, params.max_bullet);
  pnh.param("fixed_supply", params.fixed_supply, params.fixed_supply);
  pnh.param("occupy_threshold", params.occupy_threshold, params.occupy_threshold);
  pnh.param("aggressive_threshold", params.aggressive_threshold, params.aggressive_threshold);
  pnh.param("attack_threshold", params.attack_threshold, params.attack_threshold);
  pnh.param("harm_threshold_on", params.harm_threshold_on, params.harm_threshold_on);
  pnh.param("harm_threshold_off", params.harm_threshold_off, params.harm_threshold_off);

  blackboard->set("danger_hp", params.danger_hp);
  blackboard->set("sufficient_bullet", params.sufficient_bullet);
  blackboard->set("max_bullet", params.max_bullet);
  blackboard->set("fixed_supply", params.fixed_supply);
  blackboard->set("occupy_threshold", params.occupy_threshold);
  blackboard->set("aggressive_threshold", params.aggressive_threshold);
  blackboard->set("attack_threshold", params.attack_threshold);
  blackboard->set("harm_threshold_on", params.harm_threshold_on);
  blackboard->set("harm_threshold_off", params.harm_threshold_off);

  // 日志输出参数值
  ROS_INFO("Decision Parameters loaded:");
  ROS_INFO("  danger_hp=%d, sufficient_bullet=%d, max_bullet=%d",
           params.danger_hp, params.sufficient_bullet, params.max_bullet);
  ROS_INFO("  fixed_supply=%d, occupy_threshold=%d, aggressive_threshold=%d",
           params.fixed_supply, params.occupy_threshold, params.aggressive_threshold);
  ROS_INFO("  attack_threshold=%d, harm_on=%d, harm_off=%d", 
           params.attack_threshold, params.harm_threshold_on, params.harm_threshold_off);

  // Default action
  blackboard->set("action", std::string("INIT"));
  blackboard->set("goal.valid", false);
  blackboard->set("goal.cycle_index", 0);
  blackboard->set("motion_flag", 0);  // 默认为0
  blackboard->set("motion.last_flag", 0);  // 初始化motion发送记录，防止冷启动后motion不变
  blackboard->set("attack_cooldown_end_time", ros::Time(0));
  blackboard->set("central_occupiable", false);
  blackboard->set("is_enemy_occupied", false);  
  blackboard->set("central_accumulate_count", 0);
  blackboard->set("occupy_reached", false);  // 占领阈值是否到达
  blackboard->set("recover", 0);  // 回血标志，默认为0
  blackboard->set("bullet_up", 0);  // 补弹标志，默认为0
  blackboard->set("bullet_num", 0);  // 补弹数量，默认为0
  
  // Chase mode initialization
  blackboard->set("chase.target_id", uint8_t(0));
  blackboard->set("chase.target_x", 0.0f);
  blackboard->set("chase.target_y", 0.0f);
  blackboard->set("chase.initialized", false);
  
  std::string bt_xml_path;
  pnh.param<std::string>("bt_xml", bt_xml_path, std::string(""));
  if (bt_xml_path.empty())
  {
    //修改路径
    bt_xml_path = ros::package::getPath("decision_node") + "/config/strategy_tree.xml";
  }

  std::ifstream xml_file(bt_xml_path);
  if (!xml_file.is_open())
  {
    ROS_FATAL_STREAM("Failed to open bt_xml file: " << bt_xml_path);
    return 1;
  }
  std::stringstream xml_buffer;
  xml_buffer << xml_file.rdbuf();
  const std::string xml_text = xml_buffer.str();

  BT::Tree tree = factory.createTreeFromText(xml_text, blackboard);

  // 初始化时发送一次默认数据到下位机
  {
    std_msgs::UInt8 motion_msg;
    motion_msg.data = 0;   
    motion_pub.publish(motion_msg);
    
    std_msgs::UInt8 recover_msg;
    recover_msg.data = 0;   
    recover_pub.publish(recover_msg);
    
    std_msgs::UInt8 bullet_up_msg;
    bullet_up_msg.data = 0;  
    bullet_up_pub.publish(bullet_up_msg);
    
    std_msgs::UInt8 bullet_num_msg;
    bullet_num_msg.data = 0;  
    bullet_num_pub.publish(bullet_num_msg);
    
    ROS_INFO("Initialization: Sent default values - motion=0, recover=0, bullet_up=0, bullet_num=0");
  }

  ros::Rate rate(std::max(1, tick_hz));
  while (ros::ok())
  {
    ros::spinOnce();
    tree.tickRoot();
    rate.sleep();
  }

  return 0;
}
