#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "ocs2_sentry/SentryRobotInterface.h"
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <cmath>
#include <ocs2_core/misc/LoadData.h>

SentryRobotInterface::SentryRobotInterface(){}

void SentryRobotInterface::init(const std::string& taskFile)
{
    // check that task file exists
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath))
    {
        std::cerr << "[SentryRobotInterface] Loading task file: " << taskFilePath << std::endl;
    }
    else
    {
        throw std::invalid_argument("[SentryRobotInterface] Task file not found: " + taskFilePath.string());
    }
    // 默认的初始状态
    ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
    std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

    // Solver settings
    sqpSettings_ = ocs2::sqp::loadSettings(taskFile, "sqp");
    mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");
    rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout");

    setupOptimalConrolProblem(taskFile);
}

void SentryRobotInterface::setupOptimalConrolProblem(const std::string& taskFile)
{
    /*
     * ReferenceManager & SolverSynchronizedModule
     */
    referenceManagerPtr_.reset(new ocs2::ReferenceManager);
    // Cost
    ocs2::matrix_t Q(4, 4);
    ocs2::matrix_t R(2, 2);
    ocs2::matrix_t Qf(4, 4);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
    ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
    ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Qf: \n" << Qf << "\n";

    // Dynamics
    problem_.dynamicsPtr = std::make_unique<SentrySystemDynamics>();
    // Cost
    problem_.costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));  // 设置cost
    problem_.finalCostPtr->add("finalCost", std::make_unique<ocs2::QuadraticStateCost>(Qf));  // 设置finalCost
    // Rollout
    rolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings_));
    // Soft Constraints
    ocs2::RelaxedBarrierPenalty::Config barrierPenaltyConfig(0.1, 0.05);
    ocs2::RelaxedBarrierPenalty::Config barriercollisionPenaltyConfig(0.1, 0.2);
    std::unique_ptr<ocs2::StateInputSoftConstraint> stateInputSoftConstraintPtr =
            std::make_unique<ocs2::StateInputSoftConstraint>(std::make_unique<SentryStateInputConstraint>(), std::make_unique<ocs2::RelaxedBarrierPenalty>(barrierPenaltyConfig));
    problem_.softConstraintPtr->add("stateInputBounds", std::move(stateInputSoftConstraintPtr));

    ocs2::vector_t initialInput = ocs2::vector_t::Zero(2);
    initializerPtr_.reset(new ocs2::DefaultInitializer(2));
}