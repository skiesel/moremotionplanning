#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/goals/GoalState.h>
#include "SE2RigidBodyPlanning.hpp"
#include "config.hpp"

class KinematicSpatialGoal : public ompl::base::GoalState {
public:
	KinematicSpatialGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state) : ompl::base::GoalState(si) {
		setState(state);
		se2State = state_->as<ompl::base::SE2StateSpace::StateType>();
	}

	virtual double distanceGoal(const ompl::base::State *state) const {
		auto s = state->as<ompl::base::SE2StateSpace::StateType>();
		double dx = s->getX() - se2State->getX();
		double dy = s->getY() - se2State->getY();
		return sqrt(dx*dx + dy*dy);
	}
protected:
	const ompl::base::SE2StateSpace::StateType *se2State = NULL;
};

class DynamicSpatialGoal : public ompl::base::GoalState {
public:
	DynamicSpatialGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state) : ompl::base::GoalState(si) {
		setState(state);
		se2State = state_->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	}

	virtual double distanceGoal(const ompl::base::State *state) const {
		auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
		double dx = s->getX() - se2State->getX();
		double dy = s->getY() - se2State->getY();
		return sqrt(dx*dx + dy*dy);
	}
protected:
	const ompl::base::SE2StateSpace::StateType *se2State = NULL;
};

template <class Car>
BenchmarkData carBenchmark(const FileMap &params) {
	std::string which = params.stringVal("CarMap");
	Car *car = new Car();

	car->appendToName(which);

	globalParameters.globalAppBaseControl = car;

	ompl::app::SE2RigidBodyPlanning *abstract = new ompl::app::SE2RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr carPtr(car);

	ompl::base::StateSpacePtr stateSpace(carPtr->getStateSpace());

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

	ompl::base::ScopedState<ompl::base::SE2StateSpace> start(car->getGeometricComponentStateSpace());
	auto startLoc = params.doubleList("Start");

	start->setX(startLoc[0]);
	start->setY(startLoc[1]);
	start->setYaw(0);

	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(car->getGeometricComponentStateSpace());
	auto goalLoc = params.doubleList("Goal");

	goal->setX(goalLoc[0]);
	goal->setY(goalLoc[1]);
	goal->setYaw(0);

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	carPtr->addStartState(car->getFullStateFromGeometricComponent(start));
	if(params.stringVal("Domain").compare("DynamicCar") == 0) {
		auto myGoal = new DynamicSpatialGoal(
			car->getSpaceInformation(),
			car->getFullStateFromGeometricComponent(goal).get());
		myGoal->setThreshold(goalRadius);
		auto goalPtr = ompl::base::GoalPtr(myGoal);
		carPtr->setGoal(goalPtr);
	} else if(params.stringVal("Domain").compare("KinematicCar") == 0) {
		auto myGoal = new KinematicSpatialGoal(
			car->getSpaceInformation(),
			car->getFullStateFromGeometricComponent(goal).get());
		myGoal->setThreshold(goalRadius);
		auto goalPtr = ompl::base::GoalPtr(myGoal);
		carPtr->setGoal(goalPtr);
	}

	abstract->setStartAndGoalStates(start, goal, goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto agentMesh = params.stringVal("AgentMesh");
	auto environmentMesh = params.stringVal("EnvironmentMesh");

	car->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	car->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		carPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		carPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}


	if(!carPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		carPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	carPtr->setup();
	abstract->setup();

	globalParameters.abstractBounds = abstract->getStateSpace()->as<ompl::base::SE2StateSpace>()->getBounds();
	globalParameters.abstractBounds.resize(3);
	globalParameters.abstractBounds.setLow(2, -M_PI);
	globalParameters.abstractBounds.setHigh(2, M_PI);

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		ompl::base::SE2StateSpace::StateType *state = s->as<ompl::base::SE2StateSpace::StateType>();
		state->setXY(values[0], values[1]);
		state->setYaw(values[2]);
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		const ompl::base::SE2StateSpace::StateType *state = s->as<ompl::base::SE2StateSpace::StateType>();
		values.resize(3);
		values[0] = state->getX();
		values[1] = state->getY();
		values[2] = state->getYaw();
	};

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*carPtr, car->getName());
	data.simplesetup = carPtr;
	data.decomposition = car->allocDecomposition();

	return data;
}