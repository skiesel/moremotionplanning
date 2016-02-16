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

	// set the bounds for the R^3 part of SE(3)
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(0, 0);
	bounds.setLow(1, -4);
	bounds.setLow(2, -8.23);
	bounds.setHigh(0, 39.65);
	bounds.setHigh(1, 30);
	bounds.setHigh(2, 0);
	stateSpace->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(0)->setBounds(bounds);

	abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	abstract->getSpaceInformation()->setValidStateSamplerAllocator(SE3ZOnlyValidStateSamplerAllocator);

	// define start state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start(blimp->getGeometricComponentStateSpace());
	start->setX(5);
	start->setY(-2);
	start->setZ(-4);
	start->rotation().setIdentity();

	// define goal state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(blimp->getGeometricComponentStateSpace());
	goal->setX(5);
	goal->setY(28);
	goal->setZ(-4);
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

	blimp->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp.dae");
	blimp->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

	if(params.exists("MinControlDuration")) {
		blimpPtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"),
																params.integerVal("MaxControlDuration"));
	}
	if(params.exists("PropagationStepSize")) {
		blimpPtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}

	if(!blimpPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		blimpPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp.dae");
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

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

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*blimpPtr, blimp->getName());
	data.simplesetup = blimpPtr;
	data.decomposition = blimp->allocDecomposition();

	return data;
}