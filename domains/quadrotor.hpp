#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include "QuadrotorPlanning.hpp"
#include "SE3RigidBodyPlanning.hpp"
#include "config.hpp"

BenchmarkData quadrotorBenchmark() {
	ompl::app::QuadrotorPlanning *quadrotor = new ompl::app::QuadrotorPlanning();

	globalParameters.globalAppBaseControl = quadrotor;

	ompl::app::SE3RigidBodyPlanning *abstract = new ompl::app::SE3RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr quadrotorPtr(quadrotor);

	ompl::base::StateSpacePtr stateSpace(quadrotorPtr->getStateSpace());

	// set the bounds for the R^3 part of SE(3)
	ompl::base::RealVectorBounds bounds(3);
	bounds.setLow(0, 0);
	bounds.setLow(1, -4);
	bounds.setLow(2, -8.23);
	bounds.setHigh(0, 39.65);
	bounds.setHigh(1, 30);
	bounds.setHigh(2, 0);
	stateSpace->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(0)->setBounds(bounds);

	// bounds.resize();
	abstract->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
	abstract->getSpaceInformation()->setValidStateSamplerAllocator(SE3ZOnlyValidStateSamplerAllocator);

	// define start state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> start(quadrotor->getGeometricComponentStateSpace());
	start->setX(5);
	start->setY(-2);
	start->setZ(-4);
	start->rotation().setIdentity();

	// define goal state
	ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(quadrotor->getGeometricComponentStateSpace());
	goal->setX(5);
	goal->setY(28);
	goal->setZ(-4);
	goal->rotation().setIdentity();

	// set the start & goal states
	quadrotorPtr->setStartAndGoalStates(
	    quadrotor->getFullStateFromGeometricComponent(start),
	    quadrotor->getFullStateFromGeometricComponent(goal), 0.5);

	abstract->setStartAndGoalStates(start, goal, 0.5);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	quadrotor->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/quadrotor.dae");
	quadrotor->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

	quadrotorPtr->getSpaceInformation()->setMinMaxControlDuration(1, 500);
	quadrotorPtr->getSpaceInformation()->setPropagationStepSize(0.1);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/quadrotor.dae");
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

	quadrotorPtr->setup();
	abstract->setup();

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*quadrotorPtr, quadrotor->getName());
	data.simplesetup = quadrotorPtr;
	data.decomposition = quadrotor->allocDecomposition();

	return data;
}