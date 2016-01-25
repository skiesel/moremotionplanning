#pragma once

#include "bestfirstsampler.hpp"

namespace ompl {

namespace base {

class AnytimeBestFirstSampler : public ompl::base::BestFirstSampler {

public:

	AnytimeBestFirstSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                        const FileMap &params) :
		BestFirstSampler(base, start, goal, params) {}

	virtual ~AnytimeBestFirstSampler() {}

	virtual bool sample(ompl::base::State *state) {
		throw ompl::Exception("AnytimeBestFirstSampler::sample", "not implemented");
		return false;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("AnytimeBestFirstSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *, ompl::base::State *) {
		throw ompl::Exception("AnytimeBestFirstSampler::reached", "not implemented");
	}

	virtual void foundGoal(ompl::base::State *, ompl::base::State *toState, double solutionCost) {
		throw ompl::Exception("AnytimeBestFirstSampler::foundGoal", "not implemented");
	}

};

}

}