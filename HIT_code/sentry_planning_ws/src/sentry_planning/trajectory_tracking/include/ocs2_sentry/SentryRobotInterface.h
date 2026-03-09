#ifndef SENTRY_PLANNING_INTERFACE_H
#define SENTRY_PLANNING_INTERFACE_H

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include "ocs2_sentry/dynamics/SentryRobotDynamtics.h"
#include "ocs2_sentry/constraint/SentryRobotStateInputConstraint.h"
#include "ocs2_sentry/constraint/SentryRobotCollisionConstraint.h"
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

class SentryRobotInterface
{
public:

    SentryRobotInterface();
    void init(const std::string& taskFile);

    ~SentryRobotInterface() = default;

    const ocs2::vector_t& getInitialState() { return initialState_; }  /// 初始化状态

    ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }  /// MPC设置
    ocs2::sqp::Settings& sqpSettings() { return sqpSettings_; }
    void setObsCollision(ocs2::scalar_array_t timeTrajectory,
                         std::vector<std::pair<double,std::vector<Eigen::Vector3d>>> obs_points_t);

    const ocs2::OptimalControlProblem& getOptimalControlProblem() const  { return problem_; }  /// 优化问题

    std::shared_ptr<ocs2::ReferenceManager> getReferenceManagerPtr() const { return referenceManagerPtr_; }

    const ocs2::RolloutBase& getRollout() const { return *rolloutPtr_; }

    const ocs2::Initializer& getInitializer() const { return *initializerPtr_; }

    std::shared_ptr<ObsConstraintSet> obsConstraintPtr_;

    ocs2::OptimalControlProblem problem_;

private:
    void setupOptimalConrolProblem(const std::string& taskFile);

    ocs2::mpc::Settings mpcSettings_;
    ocs2::sqp::Settings sqpSettings_;
    ocs2::rollout::Settings rolloutSettings_;


    std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;  // 参考轨迹管理器

    std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;  // 动力学模型状态前向扩展
    ocs2::vector_t initialState_{4};  // 初始状态 x y v phi

    std::unique_ptr<ocs2::Initializer> initializerPtr_;



};
#endif //SENTRY_PLANNING_INTERFACE_H