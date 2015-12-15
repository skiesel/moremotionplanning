#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include "BlimpPlanning.hpp"
#include "config.hpp"

struct BenchmarkData {
    ompl::tools::Benchmark* benchmark;
    ompl::control::SimpleSetupPtr simplesetup;
    ompl::control::DecompositionPtr decomposition;
};

BenchmarkData blimpBenchmark() {
    ompl::app::BlimpPlanning *blimp = new ompl::app::BlimpPlanning();

    globalAppBaseControl = blimp;

    ompl::control::SimpleSetupPtr blimpPtr(blimp);

    ompl::base::StateSpacePtr stateSpace(blimpPtr->getStateSpace());

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, 0);
    bounds.setLow(1, -200);
    bounds.setLow(2, -324);
    bounds.setHigh(0, 1561);
    bounds.setHigh(1, 1200);
    bounds.setHigh(2, 0);
    stateSpace->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(0)->setBounds(bounds);

    // define start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(blimp->getGeometricComponentStateSpace());
    start->setX(250);
    start->setY(-100);
    start->setZ(-100);
    start->rotation().setIdentity();

    // define goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(blimp->getGeometricComponentStateSpace());
    goal->setX(250);
    goal->setY(1100);
    goal->setZ(-100);
    goal->rotation().setIdentity();

    // set the start & goal states
    blimpPtr->setStartAndGoalStates(
        blimp->getFullStateFromGeometricComponent(start),
        blimp->getFullStateFromGeometricComponent(goal), 10);

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string homeDirString(homedir);

    blimp->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp.dae");
    blimp->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/blimp_world.dae");

    blimpPtr->getSpaceInformation()->setMinMaxControlDuration(1, 500);
    blimpPtr->getSpaceInformation()->setPropagationStepSize(0.1);

    blimpPtr->setup();

    BenchmarkData data;
    data.benchmark = new ompl::tools::Benchmark(*blimpPtr, blimp->getName());
    data.simplesetup = blimpPtr;
    data.decomposition = blimp->allocDecomposition();

    return data;
}