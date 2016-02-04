#pragma once

#include "../single/eessampler.hpp"
#include "../../anytimebestfirstsampler.hpp"

namespace ompl {

namespace base {

class AnytimeEESSampler : public ompl::base::EESSampler, public ompl::base::AnytimeBestFirstSampler {
public:

	AnytimeEESSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                  const FileMap &params) :
		EESSampler(base, start, goal, params), AnytimeBestFirstSampler(base, start, goal, params) {

		incumbentCost = std::numeric_limits<double>::infinity();
		weight = std::numeric_limits<double>::infinity();
	}

	virtual ~AnytimeEESSampler() {}

	virtual void initialize() {
		EESSampler::initialize();
	}

	virtual bool sample(ompl::base::State *state) {
		return EESSampler::sample(state);
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("AnytimeEESSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *a, ompl::base::State *b) {
		EESSampler::reached(a, b);
	}

	virtual void foundGoal(ompl::base::State *, ompl::base::State *toState, double solutionCost) {
		OMPL_INFORM("GOAL");

		ompl::base::ScopedState<> incomingState(EESSampler::si_->getStateSpace());
		incomingState = toState;
		unsigned int fromIndex = EESSampler::abstraction->mapToAbstractRegion(incomingState);
		EESSampler::Vertex &reachedVertex = EESSampler::vertices[fromIndex];


		if(reachedVertex.vals[EESSampler::G] < incumbentCost) {
			double bestFValue = open.peekLeftmost()->vals[EESSampler::F];
			incumbentCost = reachedVertex.vals[EESSampler::G];
			weight = incumbentCost / bestFValue;
			heuristicCorrection = reachedVertex.vals[EESSampler::G] / solutionCost;
		}
	}

	double incumbentCost;
};

}

}
