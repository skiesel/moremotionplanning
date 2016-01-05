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


std::map<std::string, std::string> parseArgs(int argc, char *argv[]) {
	std::map<std::string, std::string> params;
	for(unsigned int i = 1; i < argc-1; i+=2) {
		std::string arg(argv[i]);
		std::string val(argv[i+1]);
		params[arg.substr(1)] = val;
	}
	return params;
}


void doBenchmarkRun(BenchmarkData &benchmarkData, std::map<std::string, std::string> &params) {
	auto planner = params["planner"];

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
		double omega = std::stod(params["omega"]);
		double stateRadius = std::stod(params["staterRadius"]);
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius));
	}
	else if(planner.compare("FBiasedShellRRT") == 0) {
		double omega = std::stod(params["omega"]);
		double stateRadius = std::stod(params["staterRadius"]);
		double shellPreference = std::stod(params["shellPreference"]);
		double shellRadius = std::stod(params["shellRadius"]);
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), omega, stateRadius, shellPreference, shellRadius));
	}
	else if(planner.compare("PlakuRRT") == 0) {
		double alpha = std::stod(params["alpha"]);
		double b = std::stod(params["b"]);
		double stateRadius = std::stod(params["staterRadius"]);
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::PlakuRRT(benchmarkData.simplesetup->getSpaceInformation(), alpha, b, stateRadius));
	}
	// else if(planner.compare("NewPlanner") == 0) {
	// 	double omega = std::stod(params["omega"]);
	// 	double stateRadius = std::stod(params["staterRadius"]);
	// 	double shellPreference = std::stod(params["shellPreference"]);
	// 	double shellRadius = std::stod(params["shellRadius"]);
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
	req.maxTime = std::stod(params["timeout"]);
	req.maxMem = std::stod(params["memory"]);
	req.runCount = std::stod(params["runs"]);
	req.displayProgress = true;

	benchmarkData.benchmark->benchmark(req);
	benchmarkData.benchmark->saveResultsToFile(params["output"].c_str());
}

int main(int argc, char *argv[]) {
	auto params = parseArgs(argc, argv);

	ompl::RNG::setSeed(std::stol(params["seed"]));

	auto domain = params["domain"];
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
		auto benchmarkData = carBenchmark<ompl::app::DynamicCarPlanning>("Polygon");
		streamPoint = stream2DPoint;
		doBenchmarkRun(benchmarkData, params);
	}
	else {
		fprintf(stderr, "unrecognized domain\n");
	}
	
	return 0;
}