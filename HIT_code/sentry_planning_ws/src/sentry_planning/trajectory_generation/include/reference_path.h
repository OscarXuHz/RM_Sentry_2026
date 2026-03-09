#ifndef SENRTY_PLANNING_REFENECESMOOTH
#define SENRTY_PLANNING_REFENECESMOOTH

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include "RM_GridMap.h"

// 求解带状线性方程组Ax=b
// A 是一个 N*N 的带状矩阵 lower band为lowerBw upper band为upperBw.
// 有O(N)的时间复杂度
class BandedSystem {
public:

    inline void create(const int &n, const int &p, const int &q) {
        // In case of re-creating before destroying
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];  // 带状矩阵所有的有效数据
        std::fill_n(ptrData, actualSize, 0.0);
        return;
    }

    inline void destroy() {
        if (ptrData != nullptr) {
            delete[] ptrData;
            ptrData = nullptr;
        }
        return;
    }

    inline void operator=(const BandedSystem &bs) {
        ptrData = nullptr;
        create(bs.N, bs.lowerBw, bs.upperBw);
        memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
    }

private:
    int N;
    int lowerBw;
    int upperBw;
    double *ptrData = nullptr;

public:
    // Reset the matrix to zero
    inline void reset(void) {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        return;
    }

    // The band matrix is stored as suggested in "Matrix Computation"
    inline const double &operator()(const int &i, const int &j) const {
        return ptrData[(i - j + upperBw) * N + j];
    }

    inline double &operator()(const int &i, const int &j) {
        return ptrData[(i - j + upperBw) * N + j];
    }

    // 进行带状 LU 分解。该函数将原始矩阵 A 转化为 LU 分解形式
    // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
    inline void factorizeLU() {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N - 2; k++) {
            iM = std::min(k + lowerBw, N - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                    operator()(i, k) /= cVl;
                }
            }
            jM = std::min(k + upperBw, N - 1);
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) {
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
        return;
    }

    // This function solves Ax=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    inline void solve(Eigen::MatrixXd &b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator ()(i, j) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        return;
    }

    // This function solves ATx=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    inline void solveAdj(Eigen::MatrixXd &b) const {
        int iM;
        for (int j = 0; j <= N - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        return;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


class Refenecesmooth {
public:
    Refenecesmooth() {};

    ~Refenecesmooth();

    std::vector<double> m_trapezoidal_time;
    std::vector<Eigen::Vector3d> reference_path;
    std::vector<Eigen::Vector3d> reference_velocity;
    Eigen::MatrixXd m_polyMatrix_x;
    Eigen::MatrixXd m_polyMatrix_y;
    BandedSystem m_bandedMatrix;
    Eigen::MatrixXd b;
    Eigen::MatrixXd b2;
    Eigen::Vector3d state_vel;


    double max_accleration = 6.4;
    double max_velocity = 4.0;
    double desire_veloity = 3.6;  /// 这里的期望速度和加速度可以控制实际运动的快慢
    double dt = 0.05;
    double traj_length = 0.0;

    double slope_coeff = 1.8;  /// TODO 这里是针对时间分配时各种地形的处理，这个参数建议移到参数表里
    double slope_coeff_xtl = 1.4;
    double slope_coeff_max = 1.0;
    double slope_coeff_xtl_max = 0.6;

    double bridge_coeff = 1.8;
    double bridge_coeff_xtl = 1.6;

    bool isxtl = false;

//    void resolve()

    void setGlobalPath(Eigen::Vector3d velocity, std::vector<Eigen::Vector2d> &global_path, double reference_amax, double desire_speed, bool xtl);

    void solveTrapezoidalTime();

    void solvePolyMatrix();

    bool checkfeasible();

    Eigen::Vector2d getRefPose();

    void getRefVel();

    Eigen::Vector2d getRefAcc();

    void getRefTrajectory(std::vector <Eigen::Vector3d> &ref_trajectory, std::vector<double> &times);
    void getTrackingTraj(int step_time, int planning_horizon, Eigen::Vector3d state, std::vector<Eigen::Vector3d> &ref_trajectory,
                         std::vector<double> &ref_phi, std::vector<double> &ref_speed);

    void getSegmentIndex(double time, int &segment_index, double &total_time);

    void reset(const Eigen::Matrix3d &headState,
               const Eigen::Matrix3d &tailState,
               const int &pieceNum);

    void reset(const int &pieceNum);
    void init(std::shared_ptr<GlobalMap> &_global_map);
    std::shared_ptr<GlobalMap> global_map;  // 地图

private:
    int N;
    Eigen::Matrix3d headPVA;
    Eigen::Matrix3d tailPVA;
    std::vector<Eigen::Vector2d> m_global_path;



};


#endif //SENRTY_PLANNING_REFENECESMOOTH
