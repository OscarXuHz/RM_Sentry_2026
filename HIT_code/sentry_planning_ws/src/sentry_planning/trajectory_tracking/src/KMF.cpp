#include <cmath>
#include "../include/KMF.h"


void KMF::initParam(double init_r, double init_q, double init_dt, bool high_order)
{
    m_default_r = init_r;
    m_default_q = init_q;
    m_default_dt = init_dt;

    m_filter_inited = false;
    if(!high_order){
        m_measure_matrix << 1, 0, 0, 0;
    }else{
        m_measure_matrix = Eigen::Matrix2d::Identity(); // H
    }
    m_R = Eigen::Matrix2d::Identity() * init_r;     // R
    m_Q << init_q * init_dt *init_dt, 0,
            0, init_q;
    Eigen::Matrix2d m_pone; // P
    m_pone << init_r, 0,
            0, init_r / init_dt;
    m_p_post = m_pone;

}

void KMF::getF(double dt)
{
    Eigen::Matrix2d fone;
    fone << 1, dt,
            0, 1;
    m_transition_matrix = fone;
}

void KMF::predictUpdate(double dt)
{
    getF(dt);
    m_state_pre = m_transition_matrix * m_state_post;
    m_p_pre = m_transition_matrix * m_p_post * m_transition_matrix.transpose() + m_Q;
}

void KMF::predictUpdate(double dt, double velocity)
{
    getF(dt);
    if(velocity<0.1)
    {
        m_Q << m_default_q * dt * dt * pow(velocity,4), 0,
                0, m_default_q * pow(velocity,4);
    }
    else
    {
        m_Q << m_default_q * dt * dt * pow(velocity,2), 0,
                0, m_default_q * pow(velocity,2);
    }

    m_state_pre = m_transition_matrix * m_state_post;
    m_p_pre = m_transition_matrix * m_p_post * m_transition_matrix.transpose() + m_Q;
}

void KMF::measureUpdate(Eigen::Vector2d measurevalues)
{
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d temp = m_measure_matrix * m_p_pre * m_measure_matrix.transpose() + m_R;
    Eigen::Matrix2d K = m_p_pre * m_measure_matrix.transpose() * temp.inverse();

    Eigen::Vector2d res = measurevalues - m_measure_matrix * m_state_pre;
    m_state_post = m_state_pre + K * res;


    m_p_post = (I - K * m_measure_matrix) * m_p_pre * (I - K * m_measure_matrix).transpose();
}

void KMF::getResults(Eigen::Vector2d &output) const
{
    output = m_state_post;
}

void KMF::setState(Eigen::Vector2d input)
{
    m_state_post = input;
}

void KMF::reset()
{
    m_filter_inited = false;
    Eigen::Matrix2d m_pone; // P
    m_pone << m_default_r, 0,
                0, m_default_r / m_default_dt;
    m_p_post = m_pone;
}