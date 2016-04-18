#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalState.h>
#include "AcrobotPlanning.hpp"
#include "SE2RigidBodyPlanning.hpp"
#include "config.hpp"

class AcrobotGoal : public ompl::base::GoalState {
public:
	AcrobotGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state) :
		ompl::base::GoalState(si) {
		setState(state);
	}

	virtual double distanceGoal(const ompl::base::State *state) const {
		auto rvs1 = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(1);
		auto rvs2 = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(2);
		return std::fabs(rvs1->values[0] + rvs2->values[0]);
	}
	
protected:
	const ompl::base::SE2StateSpace::StateType *se2State = NULL;
};

class AcrobotOptimizationObjective : public ompl::base::OptimizationObjective {
public:
	AcrobotOptimizationObjective(const ompl::base::SpaceInformationPtr &si, double maximumVelocity, double goalRadius) : OptimizationObjective(si),
		maximumVelocity(maximumVelocity), goalRadius(goalRadius) {
		setCostToGoHeuristic(boost::bind(&AcrobotOptimizationObjective::costToGoHeuristic, this, _1, _2));
	}

	ompl::base::Cost costToGoHeuristic(const ompl::base::State *a, const ompl::base::Goal *b) const {
		throw new ompl::Exception("AcrobotOptimizationObjective::costToGoHeuristic not implemented");
	}

	ompl::base::Cost stateCost(const ompl::base::State *s) const {
		throw new ompl::Exception("AcrobotOptimizationObjective::stateCost not implemented");
	}

	ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const {
		throw new ompl::Exception("AcrobotOptimizationObjective::motionCostHeuristic not implemented");
	}

	ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
		throw new ompl::Exception("AcrobotOptimizationObjective::motionCost not implemented");
	}

	double maximumVelocity, goalRadius;
};

class AcrobotValidStateSampler : public ompl::base::UniformValidStateSampler {
public:
	AcrobotValidStateSampler(const ompl::base::SpaceInformation *si) : UniformValidStateSampler(si),
		stateValidityCheckerPtr(si->getStateValidityChecker()) {}

	bool sample(ompl::base::State *state) {
		std::vector<std::vector<double>> angles = {
			{0, 0},
			{0, 0},
		};
		do {
			for(unsigned int i = 0; i < 2; i++) {
				angles[i][0] = rng.uniformReal(-M_PI, M_PI);
			}
			AcrobotValidStateSampler::acrobot->buildState(state, angles);

		} while(!stateValidityCheckerPtr->isValid(state));

		return true;
	}

	ompl::base::StateValidityCheckerPtr stateValidityCheckerPtr;
	static const ompl::app::AcrobotPlanning *acrobot;
	ompl::RNG rng;
};
const ompl::app::AcrobotPlanning *AcrobotValidStateSampler::acrobot = NULL;


ompl::base::ValidStateSamplerPtr AcrobotValidStateSamplerAllocator(const ompl::base::SpaceInformation *si) {
	return ompl::base::ValidStateSamplerPtr(new AcrobotValidStateSampler(si));
}

class AbstractAcrobotValidStateSampler : public AcrobotValidStateSampler {
public:
	AbstractAcrobotValidStateSampler(const ompl::base::SpaceInformation *si) : AcrobotValidStateSampler(si) {}

	bool sample(ompl::base::State *state) {
		ompl::base::ScopedState<> fullState(acrobot->getStateSpace());
		AcrobotValidStateSampler::sample(fullState.get());
		ompl::base::SE2StateSpace::StateType *se2 = state->as<ompl::base::SE2StateSpace::StateType>();
		ompl::base::SE2StateSpace::StateType *se2FromFull = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(2);
		se2->setXY(se2FromFull->getX(), se2FromFull->getY());
		se2->setYaw(se2FromFull->getYaw());
		return true;
	}
};

ompl::base::ValidStateSamplerPtr AbstractAcrobotValidStateSamplerAllocator(const ompl::base::SpaceInformation *si) {
	return ompl::base::ValidStateSamplerPtr(new AbstractAcrobotValidStateSampler(si));
}

BenchmarkData acrobotBenchmark(const FileMap &params) {
	ompl::app::AcrobotPlanning *acrobot = new ompl::app::AcrobotPlanning();

	AcrobotValidStateSampler::acrobot = acrobot;
	acrobot->getSpaceInformation()->setValidStateSamplerAllocator(AcrobotValidStateSamplerAllocator);

	globalParameters.globalAppBaseControl = acrobot;

	ompl::app::SE2RigidBodyPlanning *abstract = new ompl::app::SE2RigidBodyPlanning();

	abstract->getSpaceInformation()->setValidStateSamplerAllocator(AbstractAcrobotValidStateSamplerAllocator);

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr acrobotPtr(acrobot);

	ompl::base::StateSpacePtr stateSpace(acrobot->getStateSpace());

	// set the bounds for the R^2 part of SE(2)
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(0, -2);
	bounds.setHigh(0, 2);
	bounds.setLow(1, -2);
	bounds.setHigh(1, 2);

	ompl::base::RealVectorBounds bounds2(2);
	bounds2.setLow(0, -M_PI);
	bounds2.setHigh(0, M_PI);
	bounds2.setLow(1, -2*M_PI);
	bounds2.setHigh(1, 2*M_PI);

	
	for(unsigned int i = 0; i < 4; i++) {
		if(i % 2 == 0) {
			stateSpace->as<ompl::base::CompoundStateSpace>()->getSubspace(i)->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
		} else {
			stateSpace->as<ompl::base::CompoundStateSpace>()->getSubspace(i)->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds2);
		}
	}
	abstract->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

	ompl::base::ScopedState<> start = acrobot->getDefaultStartState();

	ompl::base::ScopedState<> goal = acrobot->getDefaultGoalState();

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	acrobotPtr->addStartState(start);
	auto myGoal = new AcrobotGoal(
		acrobot->getSpaceInformation(),
		goal.get());
	myGoal->setThreshold(goalRadius);
	auto goalPtr = ompl::base::GoalPtr(myGoal);
	acrobotPtr->setGoal(goalPtr);

	abstract->setStartAndGoalStates(
		start,
		goal,
		goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto environmentMesh = params.stringVal("EnvironmentMesh");

	std::string modelDir = homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/";

	acrobot->setRobotMesh(modelDir + "acrobot_link.dae");
	acrobot->addRobotMesh(modelDir + "acrobot_link.dae");

	acrobot->setEnvironmentMesh(modelDir + environmentMesh);

	abstract->setRobotMesh(modelDir + "acrobot_link.dae");
	abstract->setEnvironmentMesh(modelDir + environmentMesh);

	if(params.exists("MinControlDuration")) {
		acrobotPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}

	if(!acrobotPtr->getSpaceInformation()->getStatePropagator()->canSteer()) { //if it can steer, leave it alone!
		acrobotPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);
	}

	acrobotPtr->setup();
	abstract->setup();

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		throw ompl::Exception("Acrobot :: copyVectorToAbstractState not implemented");
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		throw ompl::Exception("Acrobot :: copyAbstractStateToVector not implemented");
	};

	double maxVel = acrobot->getMaximumTranslationalVelocity();
	globalParameters.optimizationObjective = ompl::base::OptimizationObjectivePtr(new AcrobotOptimizationObjective(blimpPtr->getSpaceInformation(), maxVel, goalRadius)));
	// blimpPtr->getProblemDefinition()->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new BlimpOptimizationObjective(blimpPtr->getSpaceInformation(), maxVel, goalRadius)));

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*acrobotPtr, acrobot->getName());
	data.simplesetup = acrobotPtr;

	data.decomposition = acrobot->allocDecomposition();

	return data;
}