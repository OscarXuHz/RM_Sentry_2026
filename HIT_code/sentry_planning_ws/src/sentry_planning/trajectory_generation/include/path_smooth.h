#ifndef SENRTY_PLANNING_PATH_SMOOTH
#define SENRTY_PLANNING_PATH_SMOOTH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <root_solver/cubic_spline.hpp>
#include <root_solver/lbfgs.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/crop_box.h>
#include "../include/RM_GridMap.h"
#include <chrono>
#include <numeric>

class Smoother{
public:


    Smoother(){};
    ~Smoother();
    std::shared_ptr<GlobalMap> global_map;
    std::vector<Eigen::Vector3d> obs_coord;
    std::vector<std::vector<Eigen::Vector3d>> allobs;
    std::vector<double> mid_distance;
    std::vector<double> weighs;
    Eigen::Matrix2Xd pathInPs;
    bool init_obs = false;
    bool init_vel = false;
    std::vector<double> m_trapezoidal_time;
    double desire_veloity;

    void setGlobalMap(std::shared_ptr<GlobalMap> &_global_map);

    void init(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel, double desire_speed);

    static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);

    void pathSample(std::vector<Eigen::Vector3d>& global_path, Eigen::Vector3d start_vel);

    void pathResample();

    std::vector<Eigen::Vector2d> getPath();

    std::vector<Eigen::Vector2d> getSamplePath();

    void smoothPath();

    void getObsEdge(Eigen::Vector2d xcur);

    Eigen::Vector2d obstacleTerm(int idx, Eigen::Vector2d xcur, double &nearest_cost);

    bool isFree(const int &idx_x, const int &idx_y, const int &idx_z) const;
    bool isFree(const Eigen::Vector3i &index) const;

    bool getObsPosition(double start_x, double start_y, double start_z,
                        double edge_x, double edge_y, double edge_z, Eigen::Vector3d &obs_pt);

    void getGuidePath(Eigen::Vector2d start_vel, double radius = 0.5);
private:
    float wObstacle = 1e5;  //不飞坡的话大一点保证安全
    int pieceN; //路径数量
    Eigen::Vector2d headP;
    Eigen::Vector2d tailP;
    CubicSpline cubSpline;
    lbfgs::lbfgs_parameter lbfgs_params;

    std::vector<Eigen::Vector2d> path;  // 采样得到的初始轨迹
    std::vector<Eigen::Vector2d> finalpath;

    static void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T);
    static void backwardT(const Eigen::VectorXd &T, Eigen::VectorXd &tau);
    static void backwardGradT(const Eigen::VectorXd &tau, Eigen::VectorXd &gradT, Eigen::VectorXd &gradTau);
};

#endif //SENRTY_PLANNING_PATH_SMOOTH