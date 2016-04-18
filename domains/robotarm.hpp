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

class RobotArmOptimizationObjective : public ompl::base::OptimizationObjective {
public:
	RobotArmOptimizationObjective(const ompl::base::SpaceInformationPtr &si, double maximumVelocity, double goalRadius) : OptimizationObjective(si),
		maximumVelocity(maximumVelocity), goalRadius(goalRadius) {
		setCostToGoHeuristic(boost::bind(&RobotArmOptimizationObjective::costToGoHeuristic, this, _1, _2));
	}

	ompl::base::Cost costToGoHeuristic(const ompl::base::State *a, const ompl::base::Goal *b) const {
		throw new ompl::Exception("RobotArmOptimizationObjective::costToGoHeuristic not implemented");
	}

	ompl::base::Cost stateCost(const ompl::base::State *s) const {
		throw new ompl::Exception("RobotArmOptimizationObjective::stateCost not implemented");
	}

	ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const {
		throw new ompl::Exception("RobotArmOptimizationObjective::motionCostHeuristic not implemented");
	}

	ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const {
		throw new ompl::Exception("RobotArmOptimizationObjective::motionCost not implemented");
	}

	double maximumVelocity, goalRadius;
};

class RobotArmValidStateSampler : public ompl::base::UniformValidStateSampler {
public:
	RobotArmValidStateSampler(const ompl::base::SpaceInformation *si) : UniformValidStateSampler(si), stateValidityCheckerPtr(si->getStateValidityChecker()) {}

	bool sample(ompl::base::State *state) {
		bool retValue = false;
		do {
			std::vector<double> jointAngles(jointBounds.low.size());
			for(unsigned int i = 0; i < jointAngles.size(); i++) {
				jointAngles[i] = rng.uniformReal(jointBounds.low[i], jointBounds.high[i]);
			}

			robotArmPlanning->buildState(jointAngles);

		} while(retValue && !stateValidityCheckerPtr->isValid(state));

		return retValue;
	}

	ompl::base::StateValidityCheckerPtr stateValidityCheckerPtr;
	ompl::RNG rng;
	static ompl::app::RobotArmPlanning *robotArmPlanning;
	static ompl::base::RealVectorBounds jointBounds;
};
ompl::app::RobotArmPlanning *RobotArmValidStateSampler::robotArmPlanning = NULL;
ompl::base::RealVectorBounds RobotArmValidStateSampler::jointBounds = ompl::base::RealVectorBounds(0);


ompl::base::ValidStateSamplerPtr RobotArmValidStateSamplerAllocator(const ompl::base::SpaceInformation *si) {
	return ompl::base::ValidStateSamplerPtr(new RobotArmValidStateSampler(si));
}

class AbstractRobotArmValidStateSampler : public ompl::base::UniformValidStateSampler {
public:
	AbstractRobotArmValidStateSampler(const ompl::base::SpaceInformation *si) : UniformValidStateSampler(si), stateValidityCheckerPtr(si->getStateValidityChecker()) {}

	bool sample(ompl::base::State *state) {
		bool retValue = false;
		do {
			retValue = UniformValidStateSampler::sample(state);
		} while(retValue && !robotArmPlanning->checkValidAbstractState(state));

		return retValue;
	}

	ompl::base::StateValidityCheckerPtr stateValidityCheckerPtr;
	ompl::RNG rng;
	static ompl::app::RobotArmPlanning *robotArmPlanning;
};
ompl::app::RobotArmPlanning *AbstractRobotArmValidStateSampler::robotArmPlanning = NULL;

ompl::base::ValidStateSamplerPtr AbstractRobotArmValidStateSamplerAllocator(const ompl::base::SpaceInformation *si) {
	return ompl::base::ValidStateSamplerPtr(new AbstractRobotArmValidStateSampler(si));
}


BenchmarkData robotArmBenchmark(const FileMap &params) {
	std::vector<std::vector<double>> transforms = {
		//base_footprint [no mesh, not really needed?]
		//base_joint [not transform and it makes more sense to start the chain with a link]
		{0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0}, //base_link
		{0, 0, 0.05315, 0, 0, 0}, //shoulder_roll_joint
		{0, 0, 0, 1.5708, 0, 0.384}, //shoulder_roll
		{0.0205, 0, 0.12435, 0, 0, 0}, //shoulder_pitch_joint
		{0, 0, 0, 1.5708, 0, 0}, //shoulder_pitch
		{-0.0215, -0.0205, 0.1255, 0, 0, 0}, //shoulder_yaw_joint
		{0, 0, 0, 1.5708, 0, -1.5708}, //shoulder_yaw
		{0.018, 0.0206, 0.1158, 0, 0, 0}, //elbow_pitch_joint
		{0, 0, 0, 1.5708, 0, 3.1416}, //elbow_pitch
		{-0.0171, -0.018, 0.09746, 0, 0, 0}, //elbow_yaw_joint
		{0, 0, 0, 1.5708, 0, 1.5708}, //elbow_yaw
		{0.02626, 0.018, 0.0718, 0, 0, 0}, //wrist_pitch_joint
		{0, 0, 0, 1.5708, 0, 1.5708}, //wrist_pitch
		{-0.026255, 0, 0.051425, 0, 0, 0}, //wrist_roll_joint
		{0, 0, 0, 3.1416, 0, -1.5708}, //wrist_roll

		//The following joints were set as free indices from the IK solver, so they're not going
		//to really matter beyond collision checking, we'll also keep their joints constant
		//so that we can maintain a static bounding box for the abstraction (for now)

		{-0.01331, 0.00485, 0.077225, 0, 0, 0}, //gripper_joint
		{0, 0, 0, 0, 0, 0}, //gripper_finger1

		//This is from the urdf file... but we're looking at this in a simplified way
		//where every link/joint is directly attached to the previous, not allowing
		//the actual tree structure that is possible
		// {0.0098, 0.00485, 0.077225, 0, 0, 0},//gripper_joint2
		{0.02311, 0, 0, 0, 0, 0,}, //gripper_joint2 -- relative to gripper_finger1
		{0, 0, 0, 0, 0, 0}, //gripper_finger2

		//There's no mesh for this, it's what we use in the IK solver for our 6DOF reference
		//{-0.002316, 0.0079, 0.079425, 0, 0, 0}, //virtual_endeffector_joint
		//{0, 0, 0, 0, 0, 0}// virtual_endeffector
	};

	std::vector<std::vector<double>> jointAxes = {
		{0, 0, 1}, //ignoring base_joint
		{0, 0, 1}, //shoulder_roll_joint
		{1, 0 ,0}, //shoulder_pitch_joint
		{0, -1, 0}, //shoulder_yaw_joint
		{1, 0, 0}, //elbow_pitch_joint
		{0, -1, 0}, //elbow_yaw_joint
		{-1, 0, 0}, //wrist_pitch_joint
		{0, 0, 1}, // wrist_roll_joint
		{1, 0, 0}, //gripper_joint -- prismatic so you REALLY better not try to move this
		{1, 0, 0}, //gripper_joint2 -- prismatic so you REALLY better not try to move this
		//virtual_endeffector_joint -- fixed
	};

	std::vector<std::vector<double>> jointRanges = {
		{0, 0}, //ignoring base_joint
		{-2.618, 2.618}, //shoulder_roll_joint
		{-1.8326, 1.8326}, //shoulder_pitch_joint
		{-1.8326, 1.8326}, //shoulder_yaw_joint
		{-1.8326, 1.8326}, //elbow_pitch_joint
		{-1.8326, 1.8326}, //elbow_yaw_joint
		{-1.8326, 1.8326}, //wrist_pitch_joint
		{-2.618, 2.618}, // wrist_roll_joint
		// {-0.008, 0.008}, //gripper_joint -- prismatic so you REALLY better not try to move this
		{0, 0},
		// {-0.008, 0.008}, //gripper_joint2 -- prismatic so you REALLY better not try to move this
		{0, 0}
		//virtual_endeffector_joint -- fixed
	};

	assert(jointAxes.size() == jointRanges.size());

	unsigned int numberOfLinks = jointAxes.size();

	ompl::base::RealVectorBounds jointBounds(jointRanges.size());
	for(unsigned int i = 0; i < jointRanges.size(); i++) {
		jointBounds.setLow(i, jointRanges[i][0]);
		jointBounds.setHigh(i, jointRanges[i][1]);
	}

	ompl::app::RobotArmPlanning *arm = new ompl::app::RobotArmPlanning(transforms, jointAxes, jointBounds);

	RobotArmValidStateSampler::robotArmPlanning = arm;
	RobotArmValidStateSampler::jointBounds = jointBounds;
	arm->getSpaceInformation()->setValidStateSamplerAllocator(RobotArmValidStateSamplerAllocator);

	globalParameters.globalAppBaseControl = arm;

	ompl::app::SE3RigidBodyPlanning *abstract = new ompl::app::SE3RigidBodyPlanning();

	AbstractRobotArmValidStateSampler::robotArmPlanning = arm;
	abstract->getSpaceInformation()->setValidStateSamplerAllocator(RobotArmValidStateSamplerAllocator);

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr armPtr(arm);

	ompl::base::StateSpacePtr stateSpace(arm->getStateSpace());

	// set the bounds for the R^3 part of SE(3)
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(0, -0.8);
	bounds.setHigh(0, 0.8);
	bounds.setLow(1, -0.8);
	bounds.setHigh(1, 0.8);
	bounds.setLow(2, -0.01);
	bounds.setHigh(2, 0.8);
	
	for(unsigned int i = 0; i < numberOfLinks; i++) {
		stateSpace->as<ompl::base::CompoundStateSpace>()->getSubspace(i)->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	}
	abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

	// define start state
	auto jointAngles = params.doubleList("StartJoinAngles");
	ompl::base::ScopedState<> start = arm->buildState(jointAngles);

	// define goal state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(arm->getGeometricComponentStateSpace(0));
	auto goalLoc = params.doubleList("Goal");
	goal->setX(goalLoc[0]);
	goal->setY(goalLoc[1]);
	goal->setZ(goalLoc[2]);
	goal->rotation().setIdentity();

	double goalRadius = params.doubleVal("GoalRadius");

	// set the start & goal states
	armPtr->addStartState(start);
	auto myGoal = new RobotArmSpatialGoal(
		arm->getSpaceInformation(),
		arm->getFullStateFromGeometricComponent(goal).get(),
		numberOfLinks);
	myGoal->setThreshold(goalRadius);
	auto goalPtr = ompl::base::GoalPtr(myGoal);
	armPtr->setGoal(goalPtr);

	abstract->setStartAndGoalStates(
		start,
		goal,
		goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto environmentMesh = params.stringVal("EnvironmentMesh");

	std::string cytonDir = homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/cyton_gamma_1500/";

	arm->setRobotMesh(cytonDir + "base.dae");
	arm->addRobotMesh(cytonDir + "shoulder_roll.dae");
	arm->addRobotMesh(cytonDir + "shoulder_pitch.dae");
	arm->addRobotMesh(cytonDir + "shoulder_yaw.dae");
	arm->addRobotMesh(cytonDir + "elbow_pitch.dae");
	arm->addRobotMesh(cytonDir + "elbow_yaw.dae");
	arm->addRobotMesh(cytonDir + "wrist_pitch.dae");
	arm->addRobotMesh(cytonDir + "wrist_roll.dae");
	arm->addRobotMesh(cytonDir + "gripper_finger1.dae");
	arm->addRobotMesh(cytonDir + "gripper_finger2.dae");

	arm->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(cytonDir + "abstract_end_effector.dae");
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		armPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		armPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}

	if(!armPtr->getSpaceInformation()->getStatePropagator()->canSteer()) { //if it can steer, leave it alone!
		armPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);
	}

	armPtr->setup();
	// abstract->setup();

	globalParameters.abstractBounds = abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->getBounds();
	globalParameters.abstractBounds.resize(6);
	globalParameters.abstractBounds.setLow(3, -M_PI);
	globalParameters.abstractBounds.setHigh(3, M_PI);
	globalParameters.abstractBounds.setLow(4, -M_PI);
	globalParameters.abstractBounds.setHigh(4, M_PI);
	globalParameters.abstractBounds.setLow(6, -M_PI);
	globalParameters.abstractBounds.setHigh(6, M_PI);

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		throw ompl::Exception("Robot Arm :: copyVectorToAbstractState not implemented");
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		throw ompl::Exception("Robot Arm :: copyAbstractStateToVector not implemented");
	};

	double maxVel = arm->getMaximumTranslationalVelocity();
	globalParameters.optimizationObjective = ompl::base::OptimizationObjectivePtr(new RobotArmOptimizationObjective(blimpPtr->getSpaceInformation(), maxVel, goalRadius)));


	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*armPtr, arm->getName());
	data.simplesetup = armPtr;

	data.decomposition = arm->allocDecomposition();

	// const ompl::base::State *st = armPtr->getProblemDefinition()->getStartState(0);
	// bool boundsBool = armPtr->getSpaceInformation()->satisfiesBounds(st);
	// bool valid = armPtr->getSpaceInformation()->isValid(st);
	// if(!boundsBool) {
	// 	fprintf(stderr, "out of bounds\n");
	// }
	// if(!valid) {
	// 	fprintf(stderr, "invalid\n");
	// }

	return data;
}