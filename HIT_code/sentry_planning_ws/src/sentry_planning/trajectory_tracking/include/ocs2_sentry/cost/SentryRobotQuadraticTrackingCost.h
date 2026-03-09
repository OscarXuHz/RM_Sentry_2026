#ifndef SENTRY_PLANNING_COST_H
#define SENTRY_PLANNING_COST_H

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/Types.h>

class SentryRobotStateInputQuadraticCost final : public ocs2::QuadraticStateInputCost {
public:
    SentryRobotStateInputQuadraticCost(ocs2::matrix_t Q, ocs2::matrix_t R);

    ~SentryRobotStateInputQuadraticCost() override = default;

    SentryRobotStateInputQuadraticCost* clone() const override { return new SentryRobotStateInputQuadraticCost(*this); }

    std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                         const ocs2::TargetTrajectories& targetTrajectories) const override;
};

class SentryRobotStateFinalQuadraticCost final : public ocs2::QuadraticStateCost {
public:
    SentryRobotStateFinalQuadraticCost(ocs2::matrix_t Q);

    ~SentryRobotStateFinalQuadraticCost() override = default;

    SentryRobotStateFinalQuadraticCost* clone() const override { return new SentryRobotStateFinalQuadraticCost(*this); }

    ocs2::vector_t getStateDeviation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                     const ocs2::TargetTrajectories& targetTrajectories) const override;
};

#endif //SENTRY_PLANNING_COST_H