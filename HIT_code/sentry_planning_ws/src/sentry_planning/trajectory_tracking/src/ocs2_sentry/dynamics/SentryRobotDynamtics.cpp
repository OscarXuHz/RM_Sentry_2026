#include "ocs2_sentry/dynamics/SentryRobotDynamtics.h"
#include <cmath>


ocs2::vector_t SentrySystemDynamics::computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input, const ocs2::PreComputation &)
{
    // state x y v phi 状态量
    // input a w 控制量
    ocs2::vector_t stateDerivative(STATE_DIM);
    stateDerivative(0) = state(2) * cos(state(3));
    stateDerivative(1) = state(2) * sin(state(3));
    stateDerivative(2) = input(0);
    stateDerivative(3) = input(1);

    return stateDerivative;
}

ocs2::VectorFunctionLinearApproximation SentrySystemDynamics::linearApproximation(ocs2::scalar_t t, const ocs2::vector_t &x, const ocs2::vector_t &u, const ocs2::PreComputation &preComp)
{
    ocs2::VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u, preComp);

    ocs2::matrix_t &dfdx = dynamics.dfdx;
    dfdx.setZero(STATE_DIM, STATE_DIM);
    dfdx(0, 2) = cos(x(3));
    dfdx(0, 3) = - x(2) * sin(x(3));
    dfdx(1, 2) = sin(x(3));
    dfdx(1, 3) = x(2) * cos(x(3));

    ocs2::matrix_t &dfdu = dynamics.dfdu;
    dfdu.setZero(STATE_DIM, INPUT_DIM);
    dfdu(2, 0) = 1;
    dfdu(3, 1) = 1;

    return dynamics;
}