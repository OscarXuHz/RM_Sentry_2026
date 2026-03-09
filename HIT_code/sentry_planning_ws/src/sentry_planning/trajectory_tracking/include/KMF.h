#ifndef RM_KMF_H
#define RM_KMF_H
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <cmath>

class KMF
{
public:
    KMF(){};
    ~KMF(){};

    bool m_filter_inited;
    void initParam(double init_r, double init_q, double init_dt = 0.01, bool high_order = true);
    void reset();
    void predictUpdate(double dt);
    void predictUpdate(double dt, double velocity);
    void measureUpdate(Eigen::Vector2d measurevalues);
    void getResults(Eigen::Vector2d &output) const;
    void setState(Eigen::Vector2d input);
    void getF(double dt);

private:
    Eigen::Vector2d m_state_pre;
    Eigen::Vector2d m_state_post;
    Eigen::Matrix2d m_p_pre;
    Eigen::Matrix2d m_p_post;
    Eigen::Matrix2d m_transition_matrix;
    Eigen::Matrix2d m_measure_matrix;
    Eigen::Matrix2d m_Q;
    Eigen::Matrix2d m_R;
    double m_default_dt;
    double m_default_r;
    double m_default_q;
};

#endif
