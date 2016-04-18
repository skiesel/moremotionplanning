#include <iostream>
#include <fstream>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/DirectedControlSampler.h>
#include "../domains/AppBase.hpp"

struct BenchmarkData {
	ompl::tools::Benchmark *benchmark;
	ompl::control::SimpleSetupPtr simplesetup;
	ompl::control::DecompositionPtr decomposition;
};

struct SolutionStream {
	void addSolution(ompl::base::Cost c, clock_t start) {
		solutions.emplace_back(c, (double)(clock()-start) / CLOCKS_PER_SEC);
	}
	std::vector<std::pair<ompl::base::Cost, double>> solutions;
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

struct GlobalParameters {
	ompl::app::AppBase<ompl::app::CONTROL> *globalAppBaseControl = NULL;
	ompl::app::AppBase<ompl::app::GEOMETRIC> *globalAbstractAppBaseGeometric = NULL;
	ompl::base::RealVectorBounds abstractBounds = ompl::base::RealVectorBounds(0);
	std::function<void(ompl::base::State*, const std::vector<double>&)> copyVectorToAbstractState;
	std::function<void(std::vector<double>&, const ompl::base::State*)> copyAbstractStateToVector;
	ompl::base::OptimizationObjectivePtr optimizationObjective;
	SolutionStream solutionStream;
};

/* simple directed controller */

unsigned int howManyControls = 1;
ompl::control::DirectedControlSamplerPtr directedControlSamplerAllocator(const ompl::control::SpaceInformation *si) {
	return ompl::control::DirectedControlSamplerPtr(new ompl::control::SimpleDirectedControlSampler(si, howManyControls));
}


/* some terrible stuff for streaming graphics information for some debugging visualization */
FILE *graphicsFileStream = stderr; //fopen("temp.vert", "w");

void streamClearScreen() {
	fprintf(graphicsFileStream, "clear\n");
}

std::function<void(const ompl::base::State *, double, double, double, double)> streamPoint;
std::function<void(const ompl::base::State *, const ompl::base::State *, double, double, double, double)> streamLine;

void stream3DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	fprintf(graphicsFileStream, "point %g %g %g %g %g %g %g\n", s->getX(), s->getY(), s->getZ(), red, green, blue, alpha);
}

void stream2DPoint(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	fprintf(graphicsFileStream, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
}

void stream2DPoint2(const ompl::base::State *state, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s = state->as<ompl::base::SE2StateSpace::StateType>();
	fprintf(graphicsFileStream, "point %g %g 0 %g %g %g %g\n", s->getX(), s->getY(), red, green, blue, alpha);
}

void stream3DLine(const ompl::base::State *state1, const ompl::base::State *state2, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s1 = state1->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	auto s2 = state2->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE3StateSpace::StateType>(0);
	fprintf(graphicsFileStream, "line %g %g %g %g %g %g %g %g %g %g\n", s1->getX(), s1->getY(), s1->getZ(), s2->getX(), s2->getY(), s2->getZ(), red, green, blue, alpha);
}

void stream2DLine(const ompl::base::State *state1, const ompl::base::State *state2, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s1 = state1->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	auto s2 = state2->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0);
	fprintf(graphicsFileStream, "line %g %g 0 %g %g 0 %g %g %g %g\n", s1->getX(), s1->getY(), s2->getX(), s2->getY(), red, green, blue, alpha);
}

void stream2DLine2(const ompl::base::State *state1, const ompl::base::State *state2, double red=1, double green=0, double blue=0, double alpha=1) {
	auto s1 = state1->as<ompl::base::SE2StateSpace::StateType>();
	auto s2 = state2->as<ompl::base::SE2StateSpace::StateType>();
	fprintf(graphicsFileStream, "line %g %g 0 %g %g 0 %g %g %g %g\n", s1->getX(), s1->getY(), s2->getX(), s2->getY(), red, green, blue, alpha);
}