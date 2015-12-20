#include "domains/AppBase.hpp"

ompl::app::AppBase<ompl::app::CONTROL> *globalAppBaseControl = NULL;

#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#include "domains/blimp.hpp"

#include "planners/fbiasedrrt.hpp"
#include "planners/fbiasedshellrrt.hpp"

int main(int argc, char *argv[]) {
    auto benchmarkData = blimpBenchmark();

    double omega = 16;
    double stateRadius = 150;
    double shellPreference = 0.8;
    double shellRadius = 500;

    auto rrt = ompl::base::PlannerPtr(new ompl::control::RRT(benchmarkData.simplesetup->getSpaceInformation()));
    auto fbiasedrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius));
    auto fbiasedshellrrt = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius));
    auto kpiece = ompl::base::PlannerPtr(new ompl::control::KPIECE1(benchmarkData.simplesetup->getSpaceInformation()));
    auto sycloprrt = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
    auto syclopest = ompl::base::PlannerPtr(new ompl::control::SyclopEST(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));

    std::vector<ompl::base::PlannerPtr> planners = {//kpiece, rrt, 
        fbiasedrrt
        //, fbiasedshellrrt, sycloprrt, syclopest
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