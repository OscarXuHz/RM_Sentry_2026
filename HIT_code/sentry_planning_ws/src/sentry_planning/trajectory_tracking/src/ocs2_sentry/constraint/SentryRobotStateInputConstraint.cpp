#include "ocs2_sentry/constraint/SentryRobotStateInputConstraint.h"
#include <cmath>
#include <iostream>

SentryStateInputConstraint::SentryStateInputConstraint()
: StateInputConstraint(ocs2::ConstraintOrder::Linear)
{
    previous_input_.setZero(INPUT_DIM);
}

ocs2::vector_t SentryStateInputConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const
{
    ocs2::vector_t constraintValue(6);
    constraintValue << -input[0] + max_input_acceleration_,
                        input[0] + max_input_acceleration_,
                       -input[1] + max_input_angular_,
                        input[1] + max_input_angular_,
                       -state[2] + max_state_velocity_,
                        state[2] + max_state_velocity_;
//                       -input[0] + previous_input_[0] + max_input_change_,
//                        input[0] - previous_input_[0] + max_input_change_;
//    previous_input_ = input;

    return constraintValue;
}

ocs2::VectorFunctionLinearApproximation SentryStateInputConstraint::getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                               const ocs2::PreComputation& preComp) const
{

    ocs2::VectorFunctionLinearApproximation constraint;
    constraint.f = getValue(time, state, input, preComp);

    constraint.dfdx.setZero(6, STATE_DIM);
    constraint.dfdx(4, 2) = -1;
    constraint.dfdx(5, 2) = 1;

    constraint.dfdu.setZero(6, INPUT_DIM);
    constraint.dfdu(0, 0) = -1;
    constraint.dfdu(1, 0) = 1;
    constraint.dfdu(2, 1) = -1;
    constraint.dfdu(3, 1) = 1;
//    dfdu(6, 0) = -1;
//    dfdu(7, 0) = 1;

    return constraint;

}