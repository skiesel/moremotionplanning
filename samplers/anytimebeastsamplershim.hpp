#pragma once

#include "anytimebeastsampler.hpp"

namespace ompl {

namespace base {

class AnytimeBeastSamplerShim : public AnytimeBeastSampler {
public:
	AnytimeBeastSamplerShim(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params, ompl::RNG &rng) :
					AnytimeBeastSampler(base, start, goal, gsr, optimizationObjective, params), sampler_(base->allocStateSampler()), goal_s(gsr), rng_(rng) {}

	~AnytimeBeastSamplerShim() {}

	virtual void initialize() {}

	virtual bool sample(ompl::base::State *to, const base::PlannerTerminationCondition &ptc) {
		if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()){
			goal_s->sampleGoal(to);
		} else {
			sampler_->sampleUniform(to);
		}

		return true;
	}

	void remove(ompl::base::State *state, double g) {}

	void foundSolution(const ompl::base::Cost &incumbent) {}

	void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {}

protected:
	ompl::base::StateSamplerPtr sampler_;
	ompl::base::GoalSampleableRegion *goal_s = nullptr;
	ompl::RNG &rng_;

	double goalBias_ = 0.05;
};

}

}
