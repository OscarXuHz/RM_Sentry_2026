#include "ocs2_sentry/cost/SentryRobotQuadraticTrackingCost.h"
#include <cmath>

SentryRobotStateInputQuadraticCost::SentryRobotStateInputQuadraticCost(ocs2::matrix_t Q, ocs2::matrix_t R)
: ocs2::QuadraticStateInputCost(std::move(Q), std::move(R)) {}

std::pair<ocs2::vector_t, ocs2::vector_t> SentryRobotStateInputQuadraticCost::getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories) const
{
    const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
    return {state - xNominal, input};
}

SentryRobotStateFinalQuadraticCost::SentryRobotStateFinalQuadraticCost(ocs2::matrix_t Q)
        : ocs2::QuadraticStateCost(std::move(Q)) {}

ocs2::vector_t SentryRobotStateFinalQuadraticCost::getStateDeviation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                                                     const ocs2::TargetTrajectories& targetTrajectories) const
{
    const ocs2::vector_t xNominal = targetTrajectories.getDesiredState(time);
    return {state - xNominal};
}
