#ifndef SENTRY_PLANNING_DYNAMICS_H
#define SENTRY_PLANNING_DYNAMICS_H

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <iostream>
#include <string>
#include <cstddef>
#include <vector>

class SentrySystemDynamics final : public ocs2::SystemDynamicsBase {
    public:
    SentrySystemDynamics() {}

    /** Destructor */
    ~SentrySystemDynamics() override = default;

    SentrySystemDynamics *clone() const override { return new SentrySystemDynamics(*this); }

    ocs2::vector_t computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t &state, const ocs2::vector_t &input, const ocs2::PreComputation &) override;

    ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t t, const ocs2::vector_t &x, const ocs2::vector_t &u, const ocs2::PreComputation &preComp) override;

    size_t STATE_DIM = 4;
    size_t INPUT_DIM = 2;
};

#endif //SENTRY_PLANNING_DYNAMICS_H