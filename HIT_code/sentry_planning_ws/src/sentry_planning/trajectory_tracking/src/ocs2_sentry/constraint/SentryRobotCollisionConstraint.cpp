#include "ocs2_sentry/constraint/SentryRobotCollisionConstraint.h"
#include <cmath>
#include <iostream>

ObsConstraintSet::ObsConstraintSet(ocs2::scalar_array_t timeTrajectory,
                                   std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> obs_points_t)
: timeTrajectory_(timeTrajectory),
  obs_points_t_(obs_points_t){}

int ObsConstraintSet::getPointsIndex(ocs2::scalar_t time) const
{
    int index = 0;
    double min_time = 1000.0;
    for (int i = 0; i < timeTrajectory_.size(); i++)
    {
        if (std::abs(timeTrajectory_[i] - time) < min_time)
        {
            min_time = std::abs(timeTrajectory_[i] - time);
            index = i;
        }
    }
    return index;
}

std::vector<std::pair<int, Eigen::Vector3d>> ObsConstraintSet::getObsPoints(int index) const
{
    return obs_points_t_[index];
}

SentryCollisionConstraint::SentryCollisionConstraint(std::shared_ptr<ObsConstraintSet>& obsConstraintPtr)
: StateConstraint(ocs2::ConstraintOrder::Quadratic)
{
    obsConstraintPtr_ = obsConstraintPtr;
}

ocs2::vector_t SentryCollisionConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const
{
    int index = obsConstraintPtr_->getPointsIndex(time);
    std::vector<std::pair<int, Eigen::Vector3d>> obs = obsConstraintPtr_->getObsPoints(index);  // 障碍物点
    size_t num_constraints = getNumConstraints(time);

    ocs2::vector_t constraintValue(num_constraints);
    for (int i = 0; i < num_constraints; i++){
        Eigen::Vector3d obs_point = obs[i].second;
        double distance = std::pow(state[0] - obs_point[0], 2) + std::pow(state[1] - obs_point[1], 2);
        if(obs[i].first == 1){
            constraintValue[i] = distance - distance_threshold_;
        }else if(obs[i].first == 0){
            constraintValue[i] = distance - distance_threshold_ - 0.15;
        }
    }

    return constraintValue;
}

size_t SentryCollisionConstraint::getNumConstraints(ocs2::scalar_t time) const
{
    int index = obsConstraintPtr_->getPointsIndex(time);
    size_t num_constraints = obsConstraintPtr_->obs_points_t_[index].size();
    return num_constraints;
}

ocs2::VectorFunctionQuadraticApproximation SentryCollisionConstraint::getQuadraticApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                               const ocs2::PreComputation& preComp) const
{
    int index = obsConstraintPtr_->getPointsIndex(time);
    std::vector<std::pair<int, Eigen::Vector3d>> obs = obsConstraintPtr_->getObsPoints(index);  // 障碍物点
    size_t num_constraints = getNumConstraints(time);

    ocs2::VectorFunctionQuadraticApproximation constraint;
    constraint.f = getValue(time, state, preComp);

    constraint.dfdx.setZero(num_constraints, STATE_DIM);
    for(int i = 0; i < num_constraints; i++){
        Eigen::Vector3d obs_point = obs[i].second;
        constraint.dfdx(i, 0) = 2 * (state[0] - obs_point[0]);
        constraint.dfdx(i, 1) = 2 * (state[1] - obs_point[1]);
    }
    constraint.dfdxx.resize(num_constraints);

    for(int i = 0; i < num_constraints; i++){
        constraint.dfdxx[i].setZero(STATE_DIM, STATE_DIM);
        constraint.dfdxx[i](0, 0) = 2;
        constraint.dfdxx[i](1, 1) = 2;
    }
    return constraint;
}