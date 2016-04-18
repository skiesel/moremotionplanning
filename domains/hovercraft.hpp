#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/goals/GoalState.h>
#include "SE2RigidBodyPlanning.hpp"
#include "config.hpp"

#include "HovercraftPlanning.hpp"

class HovercraftSpatialGoal : public ompl::base::GoalState {
public:
	HovercraftSpatialGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state) : ompl::base::GoalState(si) {
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

class HovercraftOptimizationObjective : public ompl::base::OptimizationObjective {
public:
	HovercraftOptimizationObjective(const ompl::base::SpaceInformationPtr &si, double maximumVelocity, double goalRadius) : OptimizationObjective(si),
		maximumVelocity(maximumVelocity), goalRadius(goalRadius) {
		setCostToGoHeuristic(boost::bind(&HovercraftOptimizationObjective::costToGoHeuristic, this, _1, _2));
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
		throw new ompl::Exception("HovercraftOptimizationObjective::motionCost not implemented");
		return ompl::base::Cost(0);
	}

	 double motionDistance(const ompl::base::State *s1, const ompl::base::State *s2) const {
 		const ompl::base::SE2StateSpace::StateType *se21 = s1->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
		const ompl::base::SE2StateSpace::StateType *se22 = s2->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
		double dx = se21->getX() - se22->getX();
		double dy = se21->getY() - se22->getY();

		return sqrt(dx * dx + dy * dy);
	}

	double maximumVelocity, goalRadius;
};

BenchmarkData hovercraftBenchmark(const FileMap &params) {
	ompl::app::HovercraftPlanning *hovercraft = new ompl::app::HovercraftPlanning();

	globalParameters.globalAppBaseControl = hovercraft;

	ompl::app::SE2RigidBodyPlanning *abstract = new ompl::app::SE2RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr hovercraftPtr(hovercraft);

	ompl::base::StateSpacePtr stateSpace(hovercraftPtr->getStateSpace());

	if(params.exists("EnvironmentBounds")) {
		auto boundsVals = params.doubleList("EnvironmentBounds");
		std::vector<double> low, high;
		for(unsigned int i = 0; i < boundsVals.size(); i++) {
			low.emplace_back(boundsVals[i++]);
			high.emplace_back(boundsVals[i]);
		}

		// set the bounds for the R^3 part of SE(3)
		ompl::base::RealVectorBounds bounds(2);
		for(unsigned int i = 0; i < 2; i++) {
			bounds.setLow(i, low[i]);
			bounds.setHigh(i, high[i]);
		}


		stateSpace->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE2StateSpace>(0)->setBounds(bounds);
		abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	} else {
		OMPL_WARN("using default environment bounds");
	}

	ompl::base::ScopedState<ompl::base::SE2StateSpace> start(hovercraft->getGeometricComponentStateSpace());
	auto startLoc = params.doubleList("Start");

	start->setX(startLoc[0]);
	start->setY(startLoc[1]);
	start->setYaw(0);

	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(hovercraft->getGeometricComponentStateSpace());
	auto goalLoc = params.doubleList("Goal");

	goal->setX(goalLoc[0]);
	goal->setY(goalLoc[1]);
	goal->setYaw(0);

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	hovercraftPtr->addStartState(hovercraft->getFullStateFromGeometricComponent(start));
	auto myGoal = new HovercraftSpatialGoal(
		hovercraft->getSpaceInformation(),
		hovercraft->getFullStateFromGeometricComponent(goal).get());
	myGoal->setThreshold(goalRadius);
	auto goalPtr = ompl::base::GoalPtr(myGoal);
	hovercraftPtr->setGoal(goalPtr);

	abstract->setStartAndGoalStates(start, goal, goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto agentMesh = params.stringVal("AgentMesh");
	auto environmentMesh = params.stringVal("EnvironmentMesh");

	hovercraft->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	hovercraft->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		hovercraftPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		hovercraftPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}


	if(!hovercraftPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		hovercraftPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	hovercraftPtr->setup();
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

	double maxVel = hovercraft->getMaximumTranslationalVelocity();
	globalParameters.optimizationObjective = ompl::base::OptimizationObjectivePtr(new HovercraftOptimizationObjective(hovercraftPtr->getSpaceInformation(), maxVel, goalRadius));
	// hovercraftPtr->getProblemDefinition()->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new HovercraftOptimizationObjective(hovercraftPtr->getSpaceInformation(), maxVel, goalRadius)));

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*hovercraftPtr, hovercraft->getName());
	data.simplesetup = hovercraftPtr;
	data.decomposition = hovercraft->allocDecomposition();

	return data;
}