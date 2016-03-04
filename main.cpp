#include <iostream>
#include <fstream>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/DirectedControlSampler.h>
#include "domains/AppBase.hpp"

struct BenchmarkData {
	ompl::tools::Benchmark *benchmark;
	ompl::control::SimpleSetupPtr simplesetup;
	ompl::control::DecompositionPtr decomposition;
};

struct GlobalParameters {
	ompl::app::AppBase<ompl::app::CONTROL> *globalAppBaseControl = NULL;
	ompl::app::AppBase<ompl::app::GEOMETRIC> *globalAbstractAppBaseGeometric = NULL;
	ompl::base::RealVectorBounds abstractBounds = ompl::base::RealVectorBounds(0);
	std::function<void(ompl::base::State*, const std::vector<double>&)> copyVectorToAbstractState;
	std::function<void(std::vector<double>&, const ompl::base::State*)> copyAbstractStateToVector;
};

struct Timer {
	Timer(const std::string &print) : print(print) {
		OMPL_INFORM("starting : %s", print.c_str());
		start = clock();

	}
	~Timer() {
		OMPL_INFORM("ending : %s : \t%g", print.c_str(), (double)(clock()-start) / CLOCKS_PER_SEC);
	}
	clock_t start;
	std::string print;
};

unsigned int howManyControls = 1;
ompl::control::DirectedControlSamplerPtr directedControlSamplerAllocator(const ompl::control::SpaceInformation *si) {
	return ompl::control::DirectedControlSamplerPtr(new ompl::control::SimpleDirectedControlSampler(si, howManyControls));
}

std::function<void(const ompl::base::State *, double, double, double, double)> streamPoint;

// FILE *f = fopen("temp.vert", "w");

void stream3DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	fprintf(stderr, "point %g %g %g %g %g %g %g\n", s->getX(), s->getY(), s->getZ(), red, green, blue, alpha);
	// fprintf(f, "%g %g %g %g %g %g\n", s->getX(), s->getY(), s->getZ(), red, green, blue);
}

void stream2DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	fprintf(stderr, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
	// fprintf(f, "%g %g 0 %g %g %g\n", s->getX(), s->getY(), red, green, blue);
}

void stream2DPoint2(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::SE2StateSpace::StateType>();
	fprintf(stderr, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
}

GlobalParameters globalParameters;

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/pdst/PDST.h>

#include "structs/filemap.hpp"

#include "domains/DynamicCarPlanning.hpp"
#include "domains/KinematicCarPlanning.hpp"
#include "domains/blimp.hpp"
#include "domains/quadrotor.hpp"
#include "domains/carsetup.hpp"
#include "domains/straightline.hpp"
#include "domains/hovercraft.hpp"

#include "planners/fbiasedrrt.hpp"
#include "planners/fbiasedshellrrt.hpp"
#include "planners/plakurrt.hpp"
#include "planners/RRT.hpp"
#include "planners/KPIECE.hpp"
#include "planners/bestfirstplanner.hpp"
#include "planners/anytimebestfirstplanner.hpp"
#include "planners/newplanner.hpp"


void doBenchmarkRun(BenchmarkData &benchmarkData, const FileMap &params) {
	auto planner = params.stringVal("Planner");

	ompl::base::PlannerPtr plannerPointer;
	if(planner.compare("RRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::RRT(benchmarkData.simplesetup->getSpaceInformation()));
		// plannerPointer = ompl::base::PlannerPtr(new ompl::control::RRTLocal(benchmarkData.simplesetup->getSpaceInformation()));
	} else if(planner.compare("KPIECE") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::KPIECE1(benchmarkData.simplesetup->getSpaceInformation()));
		// plannerPointer = ompl::base::PlannerPtr(new ompl::control::KPIECELocal(benchmarkData.simplesetup->getSpaceInformation()));
	} else if(planner.compare("EST") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::EST(benchmarkData.simplesetup->getSpaceInformation()));
	} else if(planner.compare("SST") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::SST(benchmarkData.simplesetup->getSpaceInformation()));;
	} else if(planner.compare("SyclopRRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	} else if(planner.compare("SyclopEST") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopEST(benchmarkData.simplesetup->getSpaceInformation(), benchmarkData.decomposition));
	} else if(planner.compare("PDST") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::PDST(benchmarkData.simplesetup->getSpaceInformation()));
	} else if(planner.compare("BestFirst") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::BestFirstPlanner(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else if(planner.compare("AnytimeBestFirst") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::AnytimeBestFirstPlanner(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else if(planner.compare("FBiasedRRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedRRT(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else if(planner.compare("FBiasedShellRRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::FBiasedShellRRT(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else if(planner.compare("PlakuRRT") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::PlakuRRT(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else if(planner.compare("NewPlanner") == 0) {
		plannerPointer = ompl::base::PlannerPtr(new ompl::control::NewPlanner(benchmarkData.simplesetup->getSpaceInformation(), params));
	} else {
		fprintf(stderr, "unrecognized planner\n");
		return;
	}

	//allow unpenalized time for precomputation -- which is logged to the output file
	plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition());
	plannerPointer->solve(0);

	if(plannerPointer->params().hasParam("intermediate_states")) {
		plannerPointer->params().setParam("intermediate_states", "true");
	}
	benchmarkData.benchmark->addPlanner(plannerPointer);

	benchmarkData.benchmark->setPostRunEvent([](const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &){
		const ompl::base::ProblemDefinitionPtr &pdef = planner->getProblemDefinition();
		if(pdef->hasExactSolution()) {
			const ompl::base::PathPtr &pp = pdef->getSolutionPath();
			if(!pp->check()) {
				fprintf(stdout, "(1) SOLUTION PATH NOT VALID\n");
				// exit(1);
			}

			double distanceToGoal = pdef->getSolutionDifference();
			ompl::base::GoalRegion *goalRegion = pdef->getGoal()->as<ompl::base::GoalRegion>();

			if(distanceToGoal > goalRegion->getThreshold()) {
				fprintf(stdout, "(2) SOLUTION PATH DOES NOT REACH GOAL\n");
				// exit(1);
			}

			ompl::geometric::PathGeometric gpp = pp->as<ompl::control::PathControl>()->asGeometric();
			ompl::base::SpaceInformationPtr si = pdef->getSpaceInformation();
			const ompl::base::StateValidityCheckerPtr &stateChecker = si->getStateValidityChecker();
			// gpp.interpolate();
			std::vector< ompl::base::State*> states = gpp.getStates();

			double maxDist = 0;
			bool valid = states.size() > 0 ? stateChecker->isValid(states[0]) : true;
			for(unsigned int i = 0; i < states.size()-1; ++i) {
				double d = si->distance(states[i], states[i+1]);
				if(d > maxDist) {
					maxDist = d;
				}
				valid = stateChecker->isValid(states[i+1]);
				if(!valid) break;
			}

			if(!valid) {
				fprintf(stdout, "(3) SOLUTION PATH IS NOT VALID\n");
				// exit(1);
			}

			// while(maxDist > 0.01) {
			// 	maxDist *= 0.5;
			// 	gpp.subdivide();
			// }

			// states = gpp.getStates();
			// for(auto *s : states) {
			// 	if(!stateChecker->isValid(s)) {
			// 		fprintf(stderr, "(4) SOLUTION PATH IS NOT VALID\n");
			// 		exit(1);
			// 	}
			// }

			// gpp.printAsMatrix(std::cout);
		} else {
			fprintf(stderr, "no solution\n");
			// exit(2);
		}
	});


	ompl::tools::Benchmark::Request req;
	req.maxTime = params.doubleVal("Timeout");
	req.maxMem = params.doubleVal("Memory");
	req.runCount = params.doubleVal("Runs");
	req.displayProgress = true;
	req.saveConsoleOutput = false;

	benchmarkData.benchmark->benchmark(req);
	benchmarkData.benchmark->saveResultsToFile(params.stringVal("Output").c_str());
}

int main(int argc, char *argv[]) {
	FileMap params(std::cin);

	ompl::RNG::setSeed(params.integerVal("Seed"));

	if(params.exists("NumControls"))
		howManyControls = params.integerVal("NumControls");

	auto domain = params.stringVal("Domain");
	if(domain.compare("Blimp") == 0) {
		auto benchmarkData = blimpBenchmark(params);
		streamPoint = stream3DPoint;
		doBenchmarkRun(benchmarkData, params);
	} else if(domain.compare("Quadrotor") == 0) {
		auto benchmarkData = quadrotorBenchmark(params);
		streamPoint = stream3DPoint;
		doBenchmarkRun(benchmarkData, params);
	} else if(domain.compare("KinematicCar") == 0) {
		auto benchmarkData = carBenchmark<ompl::app::KinematicCarPlanning>(params);
		streamPoint = stream2DPoint2;
		doBenchmarkRun(benchmarkData, params);
	} else if(domain.compare("DynamicCar") == 0) {
		auto benchmarkData = carBenchmark<ompl::app::DynamicCarPlanning>(params);
		streamPoint = stream2DPoint;
		doBenchmarkRun(benchmarkData, params);
	} else if(domain.compare("StraightLine") == 0) {
		auto benchmarkData = straightLineBenchmark(params);
		streamPoint = stream2DPoint2;
		doBenchmarkRun(benchmarkData, params);
	} else if(domain.compare("Hovercraft") == 0) {
		auto benchmarkData = hovercraftBenchmark(params);
		streamPoint = stream2DPoint;
		doBenchmarkRun(benchmarkData, params);
	} else {
		fprintf(stderr, "unrecognized domain\n");
	}

	return 0;
}
