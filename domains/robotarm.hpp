#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalState.h>
#include "RobotArmPlanning.hpp"
#include "SE3RigidBodyPlanning.hpp"
#include "config.hpp"

class RobotArmSpatialGoal : public ompl::base::GoalState {
public:
	RobotArmSpatialGoal(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state, unsigned int numberOfLinks) :
		ompl::base::GoalState(si), numberOfLinks(numberOfLinks) {
		setState(state);
		se3State = state_->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(numberOfLinks-1);
	}

	virtual double distanceGoal(const ompl::base::State *state) const {
		auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(numberOfLinks-1);
		double dx = s->getX() - se3State->getX();
		double dy = s->getY() - se3State->getY();
		double dz = s->getZ() - se3State->getZ();
		return sqrt(dx*dx + dy*dy + dz*dz);
	}
	
protected:
	const ompl::base::SE3StateSpace::StateType *se3State = NULL;
	unsigned int numberOfLinks;
};

BenchmarkData robotArmBenchmark(const FileMap &params) {
	std::vector<double> linkLengths = params.doubleList("LinkLengths");
	double jointPadding = params.doubleVal("JointPadding");
	std::vector<unsigned int> rotationAxes = params.uIntList("LinkRotationAxes");
	std::vector<double> boundsVals = params.doubleList("JointRanges");
	std::vector<double> low, high;
	for(unsigned int i = 0; i < boundsVals.size(); i++) {
		low.emplace_back(boundsVals[i++]);
		high.emplace_back(boundsVals[i]);
	}

	ompl::base::RealVectorBounds jointRanges(low.size());
	for(unsigned int i = 0; i < low.size(); i++) {
		jointRanges.setLow(i, low[i]);
		jointRanges.setHigh(i, high[i]);
	}

	ompl::app::RobotArmPlanning *arm = new ompl::app::RobotArmPlanning(linkLengths, jointPadding, rotationAxes, jointRanges);

	globalParameters.globalAppBaseControl = arm;

	ompl::app::SE3RigidBodyPlanning *abstract = new ompl::app::SE3RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr armPtr(arm);

	ompl::base::StateSpacePtr stateSpace(arm->getStateSpace());

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

	// define start state
	auto jointAngles = params.doubleList("StartJoinAngles");
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start = arm->buildStartState(jointAngles);

	// define goal state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(arm->getGeometricComponentStateSpace(0));
	auto goalLoc = params.doubleList("Goal");
	goal->setX(goalLoc[0]);
	goal->setY(goalLoc[1]);
	goal->setZ(goalLoc[2]);
	goal->rotation().setIdentity();

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	auto myGoal = new RobotArmSpatialGoal(
		arm->getSpaceInformation(),
		arm->getFullStateFromGeometricComponent(goal).get(),
		linkLengths.size());
	myGoal->setThreshold(goalRadius);
	auto goalPtr = ompl::base::GoalPtr(myGoal);
	armPtr->setGoal(goalPtr);

	abstract->setStartAndGoalStates(start, goal, goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto agentMeshes = params.stringList("AgentMeshes");
	auto environmentMesh = params.stringVal("EnvironmentMesh");

	arm->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMeshes[0]);
	for(unsigned int i = 1; i < agentMeshes.size(); i++) {
		arm->addRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMeshes[1]);
	}
	arm->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMeshes.back());
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		armPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		armPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}

	if(!armPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		armPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	armPtr->setup();
	abstract->setup();

	globalParameters.abstractBounds = abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->getBounds();
	globalParameters.abstractBounds.resize(4);
	globalParameters.abstractBounds.setLow(3, -M_PI);
	globalParameters.abstractBounds.setHigh(3, M_PI);

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		throw ompl::Exception("Robot Arm :: copyVectorToAbstractState not implemented");
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		throw ompl::Exception("Robot Arm :: copyAbstractStateToVector not implemented");
	};

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*armPtr, arm->getName());
	data.simplesetup = armPtr;
	data.decomposition = arm->allocDecomposition();

	return data;
}