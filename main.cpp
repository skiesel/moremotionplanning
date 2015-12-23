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

double omega = 16;
double stateRadius = 1;
double shellPreference = 0.85;
double shellRadius = 5;

double alpha = 0.85;
double b = 0.85;

void doBenchmarkRun(BenchmarkData &benchmarkData, std::string resultsFileName) {
	auto rrt = ompl::base::PlannerPtr(new ompl::control::RRT(benchmarkData.simplesetup->getSpaceInformation()));
	auto fbiasedrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius));
	auto fbiasedshellrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius));
	auto plakurrt = ompl::base::PlannerPtr(new ompl::control::PlakuRRT(benchmarkData.simplesetup->getSpaceInformation(), alpha, b, shellRadius));
	auto kpiece = ompl::base::PlannerPtr(new ompl::control::KPIECE1(benchmarkData.simplesetup->getSpaceInformation()));
	auto sycloprrt = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	auto syclopest = ompl::base::PlannerPtr(new ompl::control::SyclopEST(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));

	std::vector<ompl::base::PlannerPtr> planners = {
		// kpiece,
		// sycloprrt,
		// syclopest,
		// rrt,
		fbiasedrrt,
		// fbiasedshellrrt,
		// plakurrt,
	};

	ompl::tools::Benchmark::Request req;
	req.maxTime = 120.0;
	req.maxMem = 1000.0;
	req.runCount = 50;
	req.displayProgress = true;

	for(auto &planner : planners) {
		benchmarkData.benchmark->addPlanner(planner);
	}

	benchmarkData.benchmark->benchmark(req);
	benchmarkData.benchmark->saveResultsToFile(resultsFileName.c_str());
}


int main(int argc, char *argv[]) {
	auto benchmarkData = blimpBenchmark();
	doBenchmarkRun(benchmarkData, "BlimpPlanning.log");

	benchmarkData = quadrotorBenchmark();
	doBenchmarkRun(benchmarkData, "QuadrotorPlanning.log");

	benchmarkData = carBenchmark<ompl::app::KinematicCarPlanning>("Polygon");
	doBenchmarkRun(benchmarkData, "KinematicCarPlanning.log");

	benchmarkData = carBenchmark<ompl::app::DynamicCarPlanning>("Polygon");
	doBenchmarkRun(benchmarkData, "DynamicCarPlanning.log");
	
	return 0;
}