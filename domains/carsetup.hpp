#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include "SE2RigidBodyPlanning.hpp"
#include "config.hpp"

struct EnvironmentDetails {
	EnvironmentDetails(const ompl::base::StateSpacePtr &stateSpacePtr) : start(stateSpacePtr), goal(stateSpacePtr) {}

	std::string agentMesh;
	std::string environmentMesh;
	ompl::base::ScopedState<ompl::base::SE2StateSpace> start;
	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal;
};

void getRandomPolygonEnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(-1.2);
	details.start->setY(-1.0);
	details.start->setYaw(0);

	details.goal->setX(1.1);
	details.goal->setY(1.05);
	details.goal->setYaw(0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "RandomPolygons_planar_env.dae";
}

void getMazeEnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(0);
	details.start->setY(1.15);
	details.start->setYaw(0);

	details.goal->setX(0.5);
	details.goal->setY(1.15);
	details.goal->setYaw(0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "Maze_planar_env.dae";
}

void getUniqueMazeEnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(-1.1);
	details.start->setY(-1.1);
	details.start->setYaw(0);

	details.goal->setX(1.1);
	details.goal->setY(1.15);
	details.goal->setYaw(0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "UniqueSolutionMaze_env.dae";
}

void getBarriersEasyEnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(1.0);
	details.start->setY(-3.0);
	details.start->setYaw(0);

	details.goal->setX(15.0);
	details.goal->setY(-6.0);
	details.goal->setYaw(0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "Barriers_easy_env.dae";
}

void getEffortExampleEnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(-3.0);
	details.start->setY(0.0);
	details.start->setYaw(0.0);

	details.goal->setX(5.0);
	details.goal->setY(-3.0);
	details.goal->setYaw(0.0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "EffortEnvironment.dae";
}

void getEffortExample2EnvironmentDetails(EnvironmentDetails &details) {
	details.start->setX(-5.0);
	details.start->setY(2.0);
	details.start->setYaw(0.0);

	details.goal->setX(5.00);
	details.goal->setY(2.0);
	details.goal->setYaw(0.0);

	details.agentMesh = "car2_planar_robot.dae";
	details.environmentMesh = "EffortEnvironment2.dae";
}

template <class Car>
BenchmarkData carBenchmark(std::string which) {
	Car *car = new Car();

	globalParameters.globalAppBaseControl = car;

	ompl::app::SE2RigidBodyPlanning *abstract = new ompl::app::SE2RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr carPtr(car);

	ompl::base::StateSpacePtr stateSpace(carPtr->getStateSpace());

	EnvironmentDetails details(abstract->getStateSpace());

	if(which.compare("Polygon") == 0)
		getRandomPolygonEnvironmentDetails(details);
	else if(which.compare("Maze") == 0)
		getMazeEnvironmentDetails(details);
	else if(which.compare("UniqueMaze") == 0)
		getUniqueMazeEnvironmentDetails(details);
	else if(which.compare("Barriers") == 0)
		getBarriersEasyEnvironmentDetails(details);
	else if(which.compare("Effort") == 0)
		getEffortExampleEnvironmentDetails(details);
	else if(which.compare("Effort2") == 0)
		getEffortExample2EnvironmentDetails(details);

	// set the start & goal states
	carPtr->setStartAndGoalStates(
	    car->getFullStateFromGeometricComponent(details.start),
	    car->getFullStateFromGeometricComponent(details.goal), 0.5);

	abstract->setStartAndGoalStates(details.start, details.goal, 0.5);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	car->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + details.agentMesh);
	car->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/"  + details.environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + details.agentMesh);
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/"  + details.environmentMesh);

	carPtr->getSpaceInformation()->setMinMaxControlDuration(1, 100);
	// carPtr->getSpaceInformation()->setPropagationStepSize(0.1);

	carPtr->setup();
	abstract->setup();

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*carPtr, car->getName());
	data.simplesetup = carPtr;
	data.decomposition = car->allocDecomposition();

	return data;
}