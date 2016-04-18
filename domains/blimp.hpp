#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalState.h>
#include "BlimpPlanning.hpp"
#include "SE3RigidBodyPlanning.hpp"
#include "config.hpp"

class BlimpSpatialGoal : public ompl::base::GoalState {
public:
	BlimpSpatialGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state) : ompl::base::GoalState(si) {
		setState(state);
		se3State = state_->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	}

	virtual double distanceGoal(const ompl::base::State *state) const {
		auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
		double dx = s->getX() - se3State->getX();
		double dy = s->getY() - se3State->getY();
		double dz = s->getZ() - se3State->getZ();
		return sqrt(dx*dx + dy*dy + dz*dz);
	}
protected:
	const ompl::base::SE3StateSpace::StateType *se3State = NULL;
};

class BlimpOptimizationObjective : public ompl::base::OptimizationObjective {
public:
	BlimpOptimizationObjective(const ompl::base::SpaceInformationPtr &si, double maximumVelocity, double goalRadius) : OptimizationObjective(si),
		maximumVelocity(maximumVelocity), goalRadius(goalRadius) {
		setCostToGoHeuristic(boost::bind(&BlimpOptimizationObjective::costToGoHeuristic, this, _1, _2));
	}

	ompl::base::Cost costToGoHeuristic(const ompl::base::State *a, const ompl::base::Goal *b) const {
		double dist = motionDistance(a, b->as<ompl::base::GoalState>()->getState());
		return ompl::base::Cost((dist - goalRadius) / maximumVelocity);
	}

	ompl::base::Cost stateCost(const ompl::base::State *s) const {
		return ompl::base::Cost(0.);
	}

	ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const {
		double dist = motionDistance(s1, s2);
		return ompl::base::Cost(dist / maximumVelocity);
	}

	ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
		throw new ompl::Exception("BlimpOptimizationObjective::motionCost not implemented");
		return ompl::base::Cost(0);
	}

	double motionDistance(const ompl::base::State *s1, const ompl::base::State *s2) const {
		const ompl::base::SE3StateSpace::StateType *se21 = s1->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
		const ompl::base::SE3StateSpace::StateType *se22 = s2->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
		double dx = se21->getX() - se22->getX();
		double dy = se21->getY() - se22->getY();
		double dz = se21->getZ() - se22->getZ();

		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	double maximumVelocity, goalRadius;
};

ompl::base::ValidStateSamplerPtr SE3ZOnlyValidStateSamplerAllocator(const ompl::base::SpaceInformation *si) {
	class SE3ZOnlyValidStateSampler : public ompl::base::UniformValidStateSampler {
	public:
		SE3ZOnlyValidStateSampler(const ompl::base::SpaceInformation *si) : UniformValidStateSampler(si), stateValidityCheckerPtr(si->getStateValidityChecker()) {}

		bool sample(ompl::base::State *state) {
			bool retValue = false;
			do {
				retValue = UniformValidStateSampler::sample(state);
				if(retValue) {
					state->as<ompl::base::SE3StateSpace::StateType>()->rotation().setAxisAngle(0,0,1,rng.uniformReal(-M_PI, M_PI));
				}
			} while(retValue && !stateValidityCheckerPtr->isValid(state));

			return retValue;
		}

		ompl::base::StateValidityCheckerPtr stateValidityCheckerPtr;
		ompl::RNG rng;
	};


	return ompl::base::ValidStateSamplerPtr(new SE3ZOnlyValidStateSampler(si));
}


BenchmarkData blimpBenchmark(const FileMap &params) {
	ompl::app::BlimpPlanning *blimp = new ompl::app::BlimpPlanning();

	globalParameters.globalAppBaseControl = blimp;

	ompl::app::SE3RigidBodyPlanning *abstract = new ompl::app::SE3RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr blimpPtr(blimp);

	ompl::base::StateSpacePtr stateSpace(blimpPtr->getStateSpace());

	if(params.exists("EnvironmentBounds")) {
		auto boundsVals = params.doubleList("EnvironmentBounds");
		std::vector<double> low, high;
		for(unsigned int i = 0; i < boundsVals.size(); i++) {
			low.emplace_back(boundsVals[i++]);
			high.emplace_back(boundsVals[i]);
		}

		// set the bounds for the R^3 part of SE(3)
		ompl::base::RealVectorBounds bounds(3);
		for(unsigned int i = 0; i < 3; i++) {
			bounds.setLow(i, low[i]);
			bounds.setHigh(i, high[i]);
		}
		
		stateSpace->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(0)->setBounds(bounds);

		abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	} else {
		OMPL_WARN("using default environment bounds");
	}

	abstract->getSpaceInformation()->setValidStateSamplerAllocator(SE3ZOnlyValidStateSamplerAllocator);

	// define start state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start(blimp->getGeometricComponentStateSpace());
	auto startLoc = params.doubleList("Start");

	start->setX(startLoc[0]);
	start->setY(startLoc[1]);
	start->setZ(startLoc[2]);
	start->rotation().setIdentity();

	// define goal state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(blimp->getGeometricComponentStateSpace());
	auto goalLoc = params.doubleList("Goal");
	goal->setX(goalLoc[0]);
	goal->setY(goalLoc[1]);
	goal->setZ(goalLoc[2]);
	goal->rotation().setIdentity();

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	auto myGoal = new BlimpSpatialGoal(
		blimp->getSpaceInformation(),
		blimp->getFullStateFromGeometricComponent(goal).get());
	myGoal->setThreshold(goalRadius);
	auto goalPtr = ompl::base::GoalPtr(myGoal);
	blimpPtr->setGoal(goalPtr);

	abstract->setStartAndGoalStates(start, goal, goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto agentMesh = params.stringVal("AgentMesh");
	auto environmentMesh = params.stringVal("EnvironmentMesh");

	blimp->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	blimp->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		blimpPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		blimpPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}

	if(!blimpPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		blimpPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	blimpPtr->setup();
	abstract->setup();

	globalParameters.abstractBounds = abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->getBounds();
	globalParameters.abstractBounds.resize(4);
	globalParameters.abstractBounds.setLow(3, -M_PI);
	globalParameters.abstractBounds.setHigh(3, M_PI);

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		ompl::base::SE3StateSpace::StateType *state = s->as<ompl::base::SE3StateSpace::StateType>();
		state->setXYZ(values[0], values[1], values[2]);
		state->rotation().setAxisAngle(0, 0, 1, values[4]);
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		const ompl::base::SE3StateSpace::StateType *state = s->as<ompl::base::SE3StateSpace::StateType>();
		values.resize(4);
		values[0] = state->getX();
		values[1] = state->getY();
		values[2] = state->getZ();

		double x = state->rotation().x;
		double y = state->rotation().y;
		double z = state->rotation().z;
		double w = state->rotation().w;
		
		values[3] = asin(-2 * (x * z - w * y));
	};

	double maxVel = blimp->getMaximumTranslationalVelocity();
	globalParameters.optimizationObjective = ompl::base::OptimizationObjectivePtr(new BlimpOptimizationObjective(blimpPtr->getSpaceInformation(), maxVel, goalRadius));
	// blimpPtr->getProblemDefinition()->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new BlimpOptimizationObjective(blimpPtr->getSpaceInformation(), maxVel, goalRadius)));

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*blimpPtr, blimp->getName());
	data.simplesetup = blimpPtr;
	data.decomposition = blimp->allocDecomposition();

	return data;
}