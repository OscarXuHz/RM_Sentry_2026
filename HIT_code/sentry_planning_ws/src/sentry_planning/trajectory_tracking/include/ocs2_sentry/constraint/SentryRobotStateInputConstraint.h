#ifndef SENTRY_PLANNING_STATEINPUTCONSTRAINT_H
#define SENTRY_PLANNING_STATEINPUTCONSTRAINT_H

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

class SentryStateInputConstraint final : public ocs2::StateInputConstraint {
    public:
    SentryStateInputConstraint();

    ~SentryStateInputConstraint() override = default;

    SentryStateInputConstraint *clone() const override { return new SentryStateInputConstraint(*this); }

    size_t getNumConstraints(ocs2::scalar_t time) const override { return 6; };
    ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const override;

    ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                             const ocs2::PreComputation& preComp) const override;

    size_t STATE_DIM = 4;
    size_t INPUT_DIM = 2;

private:
    mutable ocs2::vector_t previous_input_;
    ocs2::scalar_t max_input_change_ = 8.0;
    ocs2::scalar_t max_input_acceleration_ = 6.0;
    ocs2::scalar_t max_input_angular_ = 7.0;
    ocs2::scalar_t max_state_velocity_ = 6.0;
};

#endif //SENTRY_PLANNING_STATEINPUTCONSTRAINT_H