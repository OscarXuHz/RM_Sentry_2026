#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <visualization_msgs/Marker.h>

ros::Subscriber path_sub_;
ros::Subscriber costmap_sub_;
ros::Publisher cmd_vel_pub_;
ros::Publisher traj_marker_pub_;
tf2_ros::Buffer* tf_buffer_;
tf2_ros::TransformListener* tf_listener_;

nav_msgs::Path latest_path_;
nav_msgs::OccupancyGrid latest_costmap_;
nav_msgs::OccupancyGrid::ConstPtr dynamic_map_msg;
bool get_dynamic_map_info = false;
bool has_path_ = false;
bool has_costmap_ = false;
std::string robot_frame_;
std::string global_frame_;

double max_speed_;
double min_speed_;
double max_yaw_rate_;
double max_accel_;
double max_dyaw_rate_;
double velocity_resolution_;
double yaw_rate_resolution_;
double dt_;
double predict_time_;
double heading_weight_;
double clearance_weight_;
double velocity_weight_;
double path_weight_;
int obstacle_check_radius_;
double safe_clearance_;

struct Trajectory {
    std::vector<geometry_msgs::Pose> path;
    double vx0, vy0;
    double score;
};

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    latest_path_ = *msg;
    has_path_ = true;
}

void record_dynamic_map_info(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    dynamic_map_msg = msg;
    get_dynamic_map_info = true;
    // 转换为 DWA 用的 costmap 格式
    latest_costmap_ = *msg;
    has_costmap_ = true;
}

std::vector<double> generateVxSamples(const geometry_msgs::Twist& current_velocity) {
    std::vector<double> samples;
    double min_v = std::max(min_speed_, current_velocity.linear.x - max_accel_ * dt_);
    double max_v = std::min(max_speed_, current_velocity.linear.x + max_accel_ * dt_);
    for (double v = min_v; v <= max_v; v += velocity_resolution_) {
        samples.push_back(v);
    }
    ROS_INFO("generateVxSamples: min_v=%.2f, max_v=%.2f, current=%.2f", min_v, max_v, current_velocity.linear.x);
    return samples;
}

std::vector<double> generateVySamples(const geometry_msgs::Twist& current_velocity) {
    std::vector<double> samples;
    double min_v = std::max(min_speed_, current_velocity.linear.y - max_accel_ * dt_);
    double max_v = std::min(max_speed_, current_velocity.linear.y + max_accel_ * dt_);
    for (double v = min_v; v <= max_v; v += velocity_resolution_) {
        samples.push_back(v);
    }
    ROS_INFO("generateVySamples: min_v=%.2f, max_v=%.2f, current=%.2f", min_v, max_v, current_velocity.linear.y);
    return samples;
}

// std::vector<double> generateKSamples(double k_min, double k_max, double k_resolution) {
//     std::vector<double> samples;
//     for (double k = k_min; k <= k_max; k += k_resolution) {
//         samples.push_back(k);
//     }
//     return samples;
// }

// Trajectory generateTrajectory(double vx0, double vy0, double k, double max_accel, double dt, double total_time) {
//     Trajectory traj;
//     traj.vx0 = vx0;
//     traj.vy0 = vy0;
//     traj.k = k;
//     double t = 0, x = 0, y = 0;
//     double vx = vx0, vy = vy0;
//     geometry_msgs::Pose pose;
//     pose.position.x = x;
//     pose.position.y = y;
//     traj.path.push_back(pose);

//     while (t < total_time) {
//         double ax = -k * vy;
//         double ay = k * vx;
//         double accel = std::sqrt(ax*ax + ay*ay);
//         if (accel > max_accel) {
//             double scale = max_accel / accel;
//             ax *= scale;
//             ay *= scale;
//         }
//         vx += ax * dt;
//         vy += ay * dt;
//         x += vx * dt;
//         y += vy * dt;
//         geometry_msgs::Pose pose;
//         pose.position.x = x;
//         pose.position.y = y;
//         traj.path.push_back(pose);
//         t += dt;
//     }
//     return traj;
// }

// double calculateClearanceScore(const Trajectory& traj, const nav_msgs::OccupancyGrid& costmap) {
//     double min_dist = INFINITY;
//     for (const auto& pose : traj.path) {
//         unsigned int mx = (pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution;
//         unsigned int my = (pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution;
//         if (mx >= costmap.info.width || my >= costmap.info.height) {
//             return 0;
//         }
//         for (int dx = -2; dx <= 2; dx++) {
//             for (int dy = -2; dy <= 2; dy++) {
//                 unsigned int nx = mx + dx;
//                 unsigned int ny = my + dy;
//                 if (nx < costmap.info.width && ny < costmap.info.height) {
//                     int index = ny * costmap.info.width + nx;
//                     if (costmap.data[index] > 50) {
//                         double dist = sqrt(dx*dx + dy*dy) * costmap.info.resolution;
//                         if (dist < min_dist) {
//                             min_dist = dist;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     if (min_dist == INFINITY) {
//         return 1.0;
//     }
//     return std::min(1.0, min_dist / 1.0);
// }

Trajectory generateTrajectory(double vx0, double vy0, double dt, double total_time) {
    Trajectory traj;
    traj.vx0 = vx0;
    traj.vy0 = vy0;
    double t = 0, x = 0, y = 0;
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    traj.path.push_back(pose);

    while (t < total_time) {
        x += vx0 * dt;
        y += vy0 * dt;
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        traj.path.push_back(pose);
        t += dt;
    }
    return traj;
}

double calculateClearanceScore(const Trajectory& traj, 
                              const nav_msgs::OccupancyGrid& costmap, 
                              const std::string& traj_frame, 
                              const std::string& costmap_frame, 
                              tf2_ros::Buffer* tf_buffer) {
    double min_dist = INFINITY;
    for (const auto& pose : traj.path) {
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header.frame_id = traj_frame;
        pose_in.header.stamp = ros::Time(0);
        pose_in.pose = pose;

        try {
            tf_buffer->transform(pose_in, pose_out, costmap_frame);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform failed in clearance check: %s", ex.what());
            return 0.0;
        }

        int mx = (pose_out.pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution;
        int my = (pose_out.pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution;
        if (mx < 0 || mx >= costmap.info.width || my < 0 || my >= costmap.info.height) {
            return 0.0; 
        }
        int index = my * costmap.info.width + mx;
        if (costmap.data[index] > 50) {
            return 0.0; 
        }
        for (int dx = -obstacle_check_radius_; dx <= obstacle_check_radius_; ++dx) {
            for (int dy = -obstacle_check_radius_; dy <= obstacle_check_radius_; ++dy) {
                int nx = mx + dx, ny = my + dy;
                if (nx >= 0 && nx < costmap.info.width && ny >= 0 && ny < costmap.info.height) {
                    if (costmap.data[ny * costmap.info.width + nx] > 50) {
                        double dist = std::hypot(dx, dy) * costmap.info.resolution;
                        if (dist < min_dist) min_dist = dist;
                    }
                }
            }
        }
    }
    return (min_dist == INFINITY) ? 1.0 : std::min(1.0, min_dist / safe_clearance_);
}


double calculatePathFollowingScore(const Trajectory& traj, const nav_msgs::Path& global_path) {
    if (traj.path.empty() || global_path.poses.empty()) return 0.0;
    double total_dist = 0.0;
    int count = 0;
    // 对轨迹上的每个点，找全局路径上最近的点，累加距离
    for (const auto& traj_pose : traj.path) {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& path_pose : global_path.poses) {
            double dx = traj_pose.position.x - path_pose.pose.position.x;
            double dy = traj_pose.position.y - path_pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        total_dist += min_dist;
        count++;
    }
    if (count == 0) return 0.0;
    double avg_dist = total_dist / count;
    // 距离越小分数越高，归一化到[0,1]
    return 1.0 / (1.0 + avg_dist);
}

double calculateHeadingScore(const Trajectory& traj, const nav_msgs::Path& global_path) {
    if (traj.path.empty() || global_path.poses.empty()) return 0.0;
    const auto& end_pose = traj.path.back();
    const auto& goal_pose = global_path.poses.back().pose;
    double dx = goal_pose.position.x - end_pose.position.x;
    double dy = goal_pose.position.y - end_pose.position.y;
    double dist = std::hypot(dx, dy);
    // 距离越近分数越高，归一化
    return 1.0 / (1.0 + dist);
}

double evaluateTrajectory(const Trajectory& traj, 
                         const nav_msgs::Path& global_path,
                         const nav_msgs::OccupancyGrid& costmap) {
    double heading_score = calculateHeadingScore(traj, global_path);
    double clearance_score = calculateClearanceScore(traj, costmap, global_frame_, latest_costmap_.header.frame_id, tf_buffer_);
    double velocity_score = sqrt(traj.vx0 * traj.vx0 + traj.vy0 * traj.vy0) / max_speed_;
    if (velocity_score > 1.0) {
        velocity_score = 1.0; // 限制速度分数在0到1之间
    }
    double path_score = calculatePathFollowingScore(traj, global_path);
    return heading_weight_ * heading_score +
           clearance_weight_ * clearance_score +
           velocity_weight_ * velocity_score +
           path_weight_ * path_score;
}

struct CmdResult {
    geometry_msgs::Twist cmd_vel_robot;
    geometry_msgs::Twist cmd_vel_map;
};

CmdResult optimizeTrajectory(
    const geometry_msgs::Pose& current_pose,
    const geometry_msgs::Twist& current_velocity,
    const nav_msgs::Path& global_path,
    const nav_msgs::OccupancyGrid& costmap) {
    std::vector<double> vx_samples = generateVxSamples(current_velocity);
    std::vector<double> vy_samples = generateVySamples(current_velocity);
    // std::vector<double> k_samples = generateKSamples(-2.0, 2.0, 0.2); // 采样范围和分辨率可调
    std::vector<Trajectory> trajectories;
    for (double vx : vx_samples) {
        for (double vy : vy_samples) {
            Trajectory traj = generateTrajectory(vx, vy, dt_, predict_time_);
            for (auto& pose : traj.path) {
                pose.position.x += current_pose.position.x;
                pose.position.y += current_pose.position.y;
            }
            traj.score = evaluateTrajectory(traj, latest_path_, latest_costmap_);
            trajectories.push_back(traj);
        }
    }
    ROS_INFO("Generated %zu trajectories", trajectories.size());

    // 可视化所有轨迹
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;  // 确保与 RViz 的 Fixed Frame 一致
    marker.header.stamp = ros::Time::now();
    marker.ns = "dwa_traj";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;  // 改用 LINE_STRIP 连接所有点
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;  // 线宽
    marker.color.a = 1.0;   // 不透明度
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // 添加轨迹点（按顺序连接）
    for (const auto& traj : trajectories) {
        for (const auto& pose : traj.path) {
            geometry_msgs::Point p;
            p.x = pose.position.x;
            p.y = pose.position.y;
            p.z = 0;  // z坐标设为0
            marker.points.push_back(p);
        }
    }
    traj_marker_pub_.publish(marker);
    CmdResult StopResult;
    geometry_msgs::Twist stop;
    StopResult.cmd_vel_robot = stop;
    StopResult.cmd_vel_map = stop;
    
    if (trajectories.empty()) {
        ROS_WARN("No valid trajectories found, stopping robot.");
        ROS_INFO("optimizeTrajectory: returning zero cmd_vel");
        return StopResult;  // 如果没有轨迹，返回停止指令
    }
    auto best_traj = *std::max_element(trajectories.begin(), trajectories.end(),
        [](const Trajectory& a, const Trajectory& b) { return a.score < b.score; });
    ROS_INFO("Best traj: vx=%.2f vy=%.2f score=%.3f", best_traj.vx0, best_traj.vy0, best_traj.score);

    geometry_msgs::Twist cmd_vel;
    double vx_map = best_traj.vx0;
    double vy_map = best_traj.vy0;
    CmdResult result;
    result.cmd_vel_map.linear.x = vx_map;
    result.cmd_vel_map.linear.y = vy_map;
    // 获取map->robot的tf
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(robot_frame_, global_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return StopResult;  // 如果没有tf，返回停止指令
    }

    // 构造map系速度向量
    tf2::Vector3 vel_map(vx_map, vy_map, 0);
    // tf变换到robot系
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);
    tf2::Vector3 vel_robot = tf_transform * vel_map;

    // 归一化方向并乘以速度模长
    double vel_norm = std::sqrt(vel_robot.x()*vel_robot.x() + vel_robot.y()*vel_robot.y());
    if (vel_norm > 1e-6) {
        double speed = std::sqrt(vx_map*vx_map + vy_map*vy_map);
        cmd_vel.linear.x = vel_robot.x() / vel_norm * speed;
        cmd_vel.linear.y = vel_robot.y() / vel_norm * speed;
    } else {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
    }
    cmd_vel.angular.z = 0;

    ROS_INFO("optimizeTrajectory: output cmd_vel vx=%.3f vy=%.3f wz=%.3f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
    result.cmd_vel_robot = cmd_vel;
    return result;
}

geometry_msgs::Twist last_cmd_vel;

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_optimizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 参数加载
    private_nh.param("max_speed", max_speed_, 0.5);
    private_nh.param("min_speed", min_speed_, -0.1);
    private_nh.param("max_yaw_rate", max_yaw_rate_, 40.0 * M_PI / 180.0);
    private_nh.param("max_accel", max_accel_, 0.2);
    private_nh.param("max_dyaw_rate", max_dyaw_rate_, 40.0 * M_PI / 180.0);
    private_nh.param("velocity_resolution", velocity_resolution_, 0.01);
    private_nh.param("yaw_rate_resolution", yaw_rate_resolution_, 0.1 * M_PI / 180.0);
    private_nh.param("dt", dt_, 0.1);
    private_nh.param("predict_time", predict_time_, 3.0);
    private_nh.param("heading_weight", heading_weight_, 0.15);
    private_nh.param("clearance_weight", clearance_weight_, 0.2);
    private_nh.param("velocity_weight", velocity_weight_, 0.2);
    private_nh.param("path_weight", path_weight_, 0.45);
    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("obstacle_check_radius", obstacle_check_radius_, 5);
    private_nh.param("safe_clearance", safe_clearance_, 0.5);


    // tf
    tf_buffer_ = new tf2_ros::Buffer();
    tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);

    // 订阅与发布
    path_sub_ = nh.subscribe("/dstar_path", 1, pathCallback);
    ros::Subscriber dynamic_map_sub = nh.subscribe("/grid", 1, record_dynamic_map_info);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    traj_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/dwa_traj_marker", 1);

    // 初始化last_cmd_vel为0
    last_cmd_vel.linear.x = 0;
    last_cmd_vel.linear.y = 0;
    last_cmd_vel.angular.z = 0;

    ros::Rate rate(1/dt_);
    while (ros::ok()) {
        ros::spinOnce();
        if (!has_path_ || !has_costmap_) {
            ROS_INFO_THROTTLE(2.0, "Waiting for path and costmap...");
            rate.sleep();
            continue;
        }
        try {
            geometry_msgs::TransformStamped transform = 
                tf_buffer_->lookupTransform(global_frame_, robot_frame_, ros::Time(0));
            geometry_msgs::Pose current_pose;
            current_pose.position.x = transform.transform.translation.x;
            current_pose.position.y = transform.transform.translation.y;
            current_pose.position.z = transform.transform.translation.z;
            current_pose.orientation = transform.transform.rotation;
            geometry_msgs::Twist current_velocity = last_cmd_vel;
            CmdResult result = optimizeTrajectory(
                current_pose, current_velocity, latest_path_, latest_costmap_);
            cmd_vel_pub_.publish(result.cmd_vel_robot);
            ROS_INFO_THROTTLE(1.0, "Published cmd_vel: vx=%.2f vy=%.2f w=%.2f", 
                result.cmd_vel_robot.linear.x, result.cmd_vel_robot.linear.y, result.cmd_vel_robot.angular.z);
            last_cmd_vel = result.cmd_vel_map; // 更新上次的map系速度
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        rate.sleep();
    }
    delete tf_buffer_;
    delete tf_listener_;
    return 0;
}