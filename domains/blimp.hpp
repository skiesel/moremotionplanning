#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include "BlimpPlanning.hpp"
#include "SE3RigidBodyPlanning.hpp"
#include "config.hpp"

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


BenchmarkData blimpBenchmark() {
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

	// set the start & goal states
	blimpPtr->setStartAndGoalStates(
	    blimp->getFullStateFromGeometricComponent(start),
	    blimp->getFullStateFromGeometricComponent(goal), 1);

	abstract->setStartAndGoalStates(start, goal, 1);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	blimp->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp.dae");
	blimp->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

	blimpPtr->getSpaceInformation()->setMinMaxControlDuration(1, 100);
	// blimpPtr->getSpaceInformation()->setPropagationStepSize(0.5);

	if(!blimpPtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		blimpPtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp.dae");
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

	blimpPtr->setup();
	abstract->setup();

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*blimpPtr, blimp->getName());
	data.simplesetup = blimpPtr;
	data.decomposition = blimp->allocDecomposition();

	return data;
}