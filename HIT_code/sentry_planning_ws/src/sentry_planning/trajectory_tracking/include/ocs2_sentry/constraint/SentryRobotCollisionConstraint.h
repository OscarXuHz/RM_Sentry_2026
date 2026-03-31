#ifndef SENTRY_PLANNING_COLLISION_CONSTRAINT_H
#define SENTRY_PLANNING_COLLISION_CONSTRAINT_H

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <memory>

class ObsConstraintSet{
public:
    ObsConstraintSet(ocs2::scalar_array_t timeTrajectory, std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> obs_points_t);
    ~ObsConstraintSet() = default;
    int getPointsIndex(ocs2::scalar_t time) const;
    std::vector<std::pair<int, Eigen::Vector3d>> getObsPoints(int index) const;
    ocs2::scalar_array_t timeTrajectory_;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> obs_points_t_;
};

class SentryCollisionConstraint final : public ocs2::StateConstraint {
    public:
    SentryCollisionConstraint(std::shared_ptr<ObsConstraintSet>& obsConstraintPtr);

    ~SentryCollisionConstraint() override = default;

    SentryCollisionConstraint *clone() const override { return new SentryCollisionConstraint(*this); }

    size_t getNumConstraints(ocs2::scalar_t time) const override;
    ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;

    ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                             const ocs2::PreComputation& preComp) const override;

    size_t STATE_DIM = 4;
    size_t INPUT_DIM = 2;

    std::shared_ptr<ObsConstraintSet> obsConstraintPtr_;

private:
    // Fix 46f: Reduced thresholds for narrow-corridor topo map.
    // Fix 44c values (0.36/0.64) combined with 0.3m map inflation required
    // 0.9m/1.1m clearance from actual walls — impossible in narrow corridors.
    // New values give 0.5m/0.7m total clearance (inflation + constraint).
    ocs2::scalar_t distance_threshold_ = 0.04;        // static: 0.2m effective, 0.5m total
    ocs2::scalar_t distance_threshold_dynamic_ = 0.16; // dynamic: 0.4m effective, 0.7m total

};

#endif //SENTRY_PLANNING_COLLISION_CONSTRAINT_H