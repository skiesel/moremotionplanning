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

int main(int argc, char *argv[]) {
	// auto benchmarkData4 = carBenchmark<ompl::app::KinematicCarPlanning>();
	// auto benchmarkData3 = carBenchmark<ompl::app::DynamicCarPlanning>();
	// auto benchmarkData2 = quadrotorBenchmark();
	auto benchmarkData = blimpBenchmark();

	double omega = 16;
	double stateRadius = 1;
	double shellPreference = 0.8;
	double shellRadius = 5;

	double alpha = 0.85;
	double b = 0.85;

	auto rrt = ompl::base::PlannerPtr(new ompl::control::RRT(benchmarkData.simplesetup->getSpaceInformation()));
	auto rrtlocal = ompl::base::PlannerPtr(new ompl::control::RRTLocal(benchmarkData.simplesetup->getSpaceInformation()));
	auto fbiasedrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius));
	auto fbiasedshellrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius));
	auto plakurrt = ompl::base::PlannerPtr(new ompl::control::PlakuRRT(benchmarkData.simplesetup->getSpaceInformation(), alpha, b, shellRadius));
	auto kpiece = ompl::base::PlannerPtr(new ompl::control::KPIECE1(benchmarkData.simplesetup->getSpaceInformation()));
	auto sycloprrt = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	auto syclopest = ompl::base::PlannerPtr(new ompl::control::SyclopEST(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));

	std::vector<ompl::base::PlannerPtr> planners = {
		kpiece,
		sycloprrt,
		syclopest,
		rrt,
		fbiasedrrt,
		fbiasedshellrrt,
        plakurrt,
	};

	for(auto &planner : planners) {
		benchmarkData.benchmark->addPlanner(planner);
	}

	ompl::tools::Benchmark::Request req;
	req.maxTime = 300.0;
	req.maxMem = 1000.0;
	req.runCount = 25;
	req.displayProgress = true;
	benchmarkData.benchmark->benchmark(req);

	benchmarkData.benchmark->saveResultsToFile();

	return 0;
}