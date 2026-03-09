//
// Created by hitcrt on 2023/4/5.
//

#ifndef SENTRY_PLANNING_LOCAL_PLANNER_H
#define SENTRY_PLANNING_LOCAL_PLANNER_H
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
//#include <root_solver/iosqp.hpp>
#include <trajectory_generation/trajectoryPoly.h>
#include "RM_GridMap.h"
#include <std_msgs/Bool.h>


#include "ocs2_sentry/SentryRobotInterface.h"
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

class LocalPlanner
{
public:
    LocalPlanner() {}
    ~LocalPlanner() {}

    void init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &_global_map);
    void reset()
    {
        m_ready_to_start = false;
        m_reach_goal = false;
        m_get_target_angle = false;
    }

    void getNMPCPredictXU(Eigen::Vector4d &predict_state);
    void getNMPCPredictionDeque(std::vector<Eigen::Vector4d> &state_deque, std::vector<Eigen::Vector2d> &input_deque);
    void rcvGlobalTrajectory(const trajectory_generation::trajectoryPolyConstPtr& polytraj);
    void getRefTrajectory();  // 获得实际的全局参考轨迹
    void getRefVel();  /// 获取全局的实际参考速度与实际的参考时间
    void getTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw);
    void getFightTrackingTraj(Eigen::Vector3d state, double time, double robot_cur_yaw);
    void linearation(double time, double robot_cur_yaw);  // 根据输入的时间进行线性插值
    int solveNMPC(Eigen::Vector4d state);
    cv::Mat swellOccMap(cv::Mat occ_map);
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);  /// 判断两点之间是否可见
    bool checkfeasible();  /// 判断MPC预测的轨迹是否可行决定是否重规划
    bool checkXtl();
    bool checkBridge();

    std::shared_ptr<GlobalMap> global_map;

    Eigen::MatrixXd m_polyMatrix_x;
    Eigen::MatrixXd m_polyMatrix_y;

    std::vector<Eigen::Vector3d> reference_path;  // 全局实际参考轨迹
    std::vector<Eigen::Vector3d> reference_velocity;  // 全局实际参考速度
    std::vector<double> duration_time;  // 接受规划结果的时间部分

    std::vector<Eigen::Vector3d> ref_trajectory;  //mpc参考轨迹
    std::vector<Eigen::Vector3d> ref_trajectory_linear;  /// 线性插值的轨迹，不是最终的参考轨迹
    std::vector<Eigen::Vector3d> ref_velocity;
    std::vector<double> ref_time;  // 全局实际时间轨迹
    std::vector<double> reference_time;  // mpc参考时间轨迹
    Eigen::Vector3d target_point;

    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> obs_points_t;

    std::vector<std::vector<Eigen::Vector3d>> obs_points;
    int obs_num = 0;

    std::vector<double> ref_phi;
    std::vector<double> ref_speed;
    ros::Subscriber reference_trajectory_sub;
    ros::Publisher redecision_pub;
    double robot_cur_pitch;
    ros::Time start_tracking_time;
    cv::Mat occ_map;
    bool m_yaw_error_flag = false;  // yaw轴偏差过大
    bool m_get_global_trajectory = false;
    int motion_mode = 0;  // 0 为正常移动模式 1 为巡航模式

    std::vector<int> tracking_low_check_flag;

private:

    std::string taskFile;
    SentryRobotInterface mpcInterface_;
    std::shared_ptr<ocs2::SqpMpc> mpcSolverPtr_;
    ocs2::SystemObservation observation;  /// 当前的状态观测量
    std::unique_ptr<ocs2::PrimalSolution> bufferPrimalSolutionPtr_;
    std::unique_ptr<ocs2::StateSoftConstraint> stateCollisionSoftConstraintPtr;

    static const int n = 4;  // state x y v phi 状态量
    static const int m = 2;  // input a w 控制量

    double m_vmax;
    double m_amax;
    double m_wmax;
    double m_jmax;

    double m_vxtl_max;   /// 小虎步的约束限制
    double m_axtl_max;
    double m_wxtl_max;
    double m_jxtl_max;

    int planning_horizon;  /// 预测视野
    double dt; // 步长
    double rho_;
    double rhoN_;
    std::vector<Eigen::Vector4d> predictState;  /// 求解结果
    std::vector<Eigen::Vector2d> predictInput;
    Eigen::Vector4d state_observe;
    std::string occ_file_path;
    double speed_direction = 1;


    Eigen::Matrix<double, n, n> state_transition; /// 离散状态转移矩阵
    Eigen::Matrix<double, n, m> input_matrix;  /// 离散控制输入矩阵
    Eigen::Vector4d gd;
    Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // 状态约束矩阵
    Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // 控制量约束矩阵
    Eigen::SparseMatrix<double> Qweight;  // 跟踪权重矩阵
    Eigen::SparseMatrix<double> Rweight;  // 平滑权重矩阵
    Eigen::SparseMatrix<double> RSmooth;

    Eigen::SparseMatrix<double> osqpcost_quadratic;  /// osqp 损失函数矩阵
    Eigen::SparseMatrix<double> osqpcost_linear;
    Eigen::SparseMatrix<double> osqpconstraint_matrix;  /// osqp的约束输入矩阵
    Eigen::SparseMatrix<double> osqpupper_bound;
    Eigen::SparseMatrix<double> osqplower_bound;

    bool m_ready_to_start = false;
    bool m_reach_goal = false;
    bool m_get_target_angle = false;
    double m_last_target_angle;
    double m_last_delta_yaw;
    int m_delta_yaw_flag = 1;  // 速度方向

    void getSegmentIndex(double time, int &segment_index, double &total_time);  // 根据输入的时间判断在哪个区间，用于全局获取参考轨迹和速度
    void getTrackingSegmentIndex(double time, int &segment_index, double &total_time); // 根据输入时间判断在哪个区间，用于获取MPC参考轨迹速度
    void getCircleEvadeTraj(Eigen::Vector3d cur_position, std::vector<Eigen::Vector3d> &ref_trajectory, std::vector<Eigen::Vector3d> &ref_velocity, std::vector<double> &reference_time);
//    double trapezoidal_plan(double theta_start, double theta_end, double w_max, double w_acc, double d, double l);
};

#endif // SENTRY_PLANNING_LOCAL_PLANNER_H