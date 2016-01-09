#include <iostream>
#include <fstream>

#include <ompl/tools/benchmark/Benchmark.h>
#include "domains/AppBase.hpp"

struct BenchmarkData {
	ompl::tools::Benchmark *benchmark;
	ompl::control::SimpleSetupPtr simplesetup;
	ompl::control::DecompositionPtr decomposition;
};

struct GlobalParameters {
	ompl::app::AppBase<ompl::app::CONTROL> *globalAppBaseControl = NULL;
	ompl::app::AppBase<ompl::app::GEOMETRIC> *globalAbstractAppBaseGeometric = NULL;
};

std::function<void(const ompl::base::State*, double, double, double, double)> streamPoint;

void stream3DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	fprintf(stderr, "point %g %g %g %g %g %g %g\n", s->getX(), s->getY(), s->getZ(), red, green, blue, alpha);
}

void stream2DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	fprintf(stderr, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
}

void stream2DPoint2(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::SE2StateSpace::StateType>();
	fprintf(stderr, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
}

GlobalParameters globalParameters;

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#include "domains/DynamicCarPlanning.hpp"
#include "domains/KinematicCarPlanning.hpp"

#include "domains/blimp.hpp"
#include "domains/quadrotor.hpp"
#include "domains/carsetup.hpp"

#include "planners/fbiasedrrt.hpp"
#include "planners/fbiasedshellrrt.hpp"
#include "planners/plakurrt.hpp"
#include "planners/RRT.hpp"
#include "planners/KPIECE.hpp"
// #include "planners/newplanner.hpp"

#include "structs/filemap.hpp"


void doBenchmarkRun(BenchmarkData &benchmarkData, const FileMap &params) {
	auto planner = params.stringVal("Planner");

	ompl::base::PlannerPtr plannerPointer;
	if(planner.compare("RRT") == 0) {
		// plannerPointer = ompl::base::PlannerPtr(new ompl::control::RRT(benchmarkData.simplesetup->getSpaceInformation()));
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::RRTLocal(benchmarkData.simplesetup->getSpaceInformation()));
	}
	else if(planner.compare("KPIECE") == 0) {
		// plannerPointer = ompl::base::PlannerPtr(new ompl::control::KPIECE1(benchmarkData.simplesetup->getSpaceInformation()));
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::KPIECELocal(benchmarkData.simplesetup->getSpaceInformation()));
	}
	else if(planner.compare("SyclopRRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	}
	else if(planner.compare("SyclopEST") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopEST(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	}
	else if(planner.compare("FBiasedRRT") == 0) {
		double omega = params.doubleVal("Omega");
		double stateRadius = params.doubleVal("StateRadius");
		bool cheat = params.exists("Cheat") && params.stringVal("Cheat").compare("true") == 0;
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, cheat));
		if(cheat) { plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition()); plannerPointer->solve(0); }
	}
	else if(planner.compare("FBiasedShellRRT") == 0) {
		double omega = params.doubleVal("Omega");
		double stateRadius = params.doubleVal("StateRadius");
		double shellPreference = params.doubleVal("ShellPreference");
		double shellRadius = params.doubleVal("ShellRadius");
		bool cheat = params.exists("Cheat") && params.stringVal("Cheat").compare("true") == 0;
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius, cheat));
		if(cheat) { plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition()); plannerPointer->solve(0); }
	}
	else if(planner.compare("PlakuRRT") == 0) {
		double alpha = params.doubleVal("Alpha");
		double b = params.doubleVal("B");
		double stateRadius = params.doubleVal("StateRadius");
		bool cheat = params.exists("Cheat") && params.stringVal("Cheat").compare("true") == 0;
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::PlakuRRT(benchmarkData.simplesetup->getSpaceInformation(), alpha, b, stateRadius, cheat));
		if(cheat) { plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition()); plannerPointer->solve(0); }
	}
	// else if(planner.compare("NewPlanner") == 0) {
	// 	double omega = params.doubleVal("Omega");
	// 	double stateRadius = params.doubleVal("StaterRadius");
	// 	double shellPreference = params.doubleVal("ShellPreference");
	// 	double shellRadius = params.doubleVal("ShellRadius");
	// 	plannerPointer = ompl::base::PlannerPtr(new ompl::control::NewPlanner(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius));
	// }
	else {
		fprintf(stderr, "unrecognized planner\n");
		return;
	}

	if(plannerPointer->params().hasParam("intermediate_states")) {
		plannerPointer->params().setParam("intermediate_states", "true");
	}
	benchmarkData.benchmark->addPlanner(plannerPointer);

	ompl::tools::Benchmark::Request req;
	req.maxTime = params.doubleVal("Timeout");
	req.maxMem = params.doubleVal("Memory");
	req.runCount = params.doubleVal("Runs");
	req.displayProgress = true;

	benchmarkData.benchmark->benchmark(req);
	benchmarkData.benchmark->saveResultsToFile(params.stringVal("Output").c_str());
}

int main(int argc, char *argv[]) {
	FileMap params(std::cin);

	ompl::RNG::setSeed(params.integerVal("Seed"));

	auto domain = params.stringVal("Domain");
	if(domain.compare("Blimp") == 0) {
		auto benchmarkData = blimpBenchmark();
		streamPoint = stream3DPoint;
		doBenchmarkRun(benchmarkData, params);
	}
	else if(domain.compare("Quadrotor") == 0) {
		auto benchmarkData = quadrotorBenchmark();
		streamPoint = stream3DPoint;
		doBenchmarkRun(benchmarkData, params);
	}
	else if(domain.compare("KinematicCar") == 0) {
		auto benchmarkData = carBenchmark<ompl::app::KinematicCarPlanning>("Polygon");
		streamPoint = stream2DPoint2;
		doBenchmarkRun(benchmarkData, params);
	}
	else if(domain.compare("DynamicCar") == 0) {
		auto benchmarkData = carBenchmark<ompl::app::DynamicCarPlanning>("Barriers");
		streamPoint = stream2DPoint;
		doBenchmarkRun(benchmarkData, params);
	}
	else {
		fprintf(stderr, "unrecognized domain\n");
	}
	
	return 0;
}