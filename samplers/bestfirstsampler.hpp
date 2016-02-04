#pragma once

#include "fbiasedstatesampler.hpp"

namespace ompl {

namespace base {

class BestFirstSampler : public ompl::base::FBiasedStateSampler {
public:
	BestFirstSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                 const FileMap &params) :
		FBiasedStateSampler(base, start, goal, params), params(params), randomStateProbability(params.doubleVal("RandomStateProbability")),
		peekPenalty(params.doubleVal("PeekPenalty")) {}

	virtual ~BestFirstSampler() {}

	virtual bool sample(ompl::base::State *state) {
		throw ompl::Exception("BestFirstSampler::sample", "not implemented");
		return false;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("BestFirstSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *, ompl::base::State *) {
		throw ompl::Exception("BestFirstSampler::reached", "not implemented");
	}

protected:

	const FileMap &params;
	ompl::RNG randomNumbers;
	double randomStateProbability, peekPenalty;

};

}

}