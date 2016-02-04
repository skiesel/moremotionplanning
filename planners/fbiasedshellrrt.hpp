#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "../structs/filemap.hpp"

#include "../samplers/fbiasedshellstatesampler.hpp"
#include <limits>

namespace ompl {

namespace control {

class FBiasedShellRRT : public ompl::control::RRT {
public:

	/** \brief Constructor */
	FBiasedShellRRT(const SpaceInformationPtr &si, const FileMap &params) :
		ompl::control::RRT(si), shellsampler_(NULL), params(params) {

		setName("FBiased RRT Shell");

		Planner::declareParam<double>("omega", this, &FBiasedShellRRT::ignoreSetterDouble, &FBiasedShellRRT::getOmega);
		Planner::declareParam<double>("state_radius", this, &FBiasedShellRRT::ignoreSetterDouble, &FBiasedShellRRT::getStateRadius);
		Planner::declareParam<double>("shell_depth", this, &FBiasedShellRRT::ignoreSetterDouble, &FBiasedShellRRT::getShellDepth);
		Planner::declareParam<double>("shell_preference", this, &FBiasedShellRRT::ignoreSetterDouble, &FBiasedShellRRT::getShellPreference);
		Planner::declareParam<double>("prm_size", this, &FBiasedShellRRT::ignoreSetterUnsigedInt, &FBiasedShellRRT::getPRMSize);
		Planner::declareParam<double>("num_prm_edges", this, &FBiasedShellRRT::ignoreSetterUnsigedInt, &FBiasedShellRRT::getNumPRMEdges);

		//Obviously this isn't really a parameter but I have no idea how else to get it into the output file through the benchmarker
		Planner::declareParam<double>("sampler_initialization_time", this, &FBiasedShellRRT::ignoreSetterDouble, &FBiasedShellRRT::getSamplerInitializationTime);
	}

	virtual ~FBiasedShellRRT() {}

	void ignoreSetterDouble(double) const {}
	void ignoreSetterUnsigedInt(unsigned int) const {}

	double getOmega() const {
		return params.doubleVal("Omega");
	}
	double getStateRadius() const {
		return params.doubleVal("StateRadius");
	}
	double getShellDepth() const {
		return params.doubleVal("ShellDepth");
	}
	double getShellPreference() const {
		return params.doubleVal("ShellPreference");
	}
	unsigned int getPRMSize() const {
		return params.integerVal("PRMSize");
	}
	unsigned int getNumPRMEdges() const {
		return params.integerVal("NumEdges");
	}
	double getSamplerInitializationTime() const {
		return samplerInitializationTime;
	}

	/** \brief Continue solving for some amount of time. Return true if solution was found. */
	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		checkValidity();
		base::Goal                   *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

		while(const base::State *st = pis_.nextStart()) {
			Motion *motion = new Motion(siC_);
			si_->copyState(motion->state, st);
			siC_->nullControl(motion->control);
			nn_->add(motion);
		}

		if(nn_->size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		if(!shellsampler_) {
			auto start = clock();

			shellsampler_ = new ompl::base::FBiasedShellStateSampler((ompl::base::SpaceInformation *)siC_, pdef_->getStartState(0), pdef_->getGoal(),
			        params);
			shellsampler_->initialize();

			samplerInitializationTime = (double)(clock() - start) / CLOCKS_PER_SEC;
		}
		if(!controlSampler_)
			controlSampler_ = siC_->allocDirectedControlSampler();

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

		Motion *solution  = NULL;
		Motion *approxsol = NULL;
		double  approxdif = std::numeric_limits<double>::infinity();

		Motion      *rmotion = new Motion(siC_);
		base::State  *rstate = rmotion->state;
		Control       *rctrl = rmotion->control;
		base::State  *xstate = si_->allocState();

		while(ptc == false) {
			/* sample random state (with goal biasing) */
			if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
				goal_s->sampleGoal(rstate);
			else {
				shellsampler_->sample(rstate);
			}

#ifdef STREAM_GRAPHICS
			streamPoint(rmotion->state, 0, 1, 0, 1);
#endif

			/* find closest state in the tree */
			Motion *nmotion = nn_->nearest(rmotion);

			/* sample a random control that attempts to go towards the random state, and also sample a control duration */
			unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

			if(addIntermediateStates_) {
				// this code is contributed by Jennifer Barry
				std::vector<base::State *> pstates;
				cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

				if(cd >= siC_->getMinControlDuration()) {
					Motion *lastmotion = nmotion;
					bool solved = false;
					size_t p = 0;
					for(; p < pstates.size(); ++p) {
						/* create a motion */
						Motion *motion = new Motion();
						motion->state = pstates[p];

						shellsampler_->reached(pstates[p]);

#ifdef STREAM_GRAPHICS
						streamPoint(pstates[p], 1, 0, 0, 1);
#endif

						//we need multiple copies of rctrl
						motion->control = siC_->allocControl();
						siC_->copyControl(motion->control, rctrl);
						motion->steps = 1;
						motion->parent = lastmotion;
						lastmotion = motion;
						nn_->add(motion);
						double dist = 0.0;
						solved = goal->isSatisfied(motion->state, &dist);
						if(solved) {
							approxdif = dist;
							solution = motion;
							break;
						}
						if(dist < approxdif) {
							approxdif = dist;
							approxsol = motion;
						}
					}

					//free any states after we hit the goal
					while(++p < pstates.size())
						si_->freeState(pstates[p]);
					if(solved)
						break;
				} else
					for(size_t p = 0 ; p < pstates.size(); ++p)
						si_->freeState(pstates[p]);
			} else {
				if(cd >= siC_->getMinControlDuration()) {
					/* create a motion */
					Motion *motion = new Motion(siC_);

					si_->copyState(motion->state, rmotion->state);
					siC_->copyControl(motion->control, rctrl);
					motion->steps = cd;
					motion->parent = nmotion;

					shellsampler_->reached(motion->state);

#ifdef STREAM_GRAPHICS
					streamPoint(nmotion->state, 1, 0, 0, 1);
					streamPoint(motion->state, 1, 0, 0, 1);
#endif

					nn_->add(motion);
					double dist = 0.0;
					bool solv = goal->isSatisfied(motion->state, &dist);
					if(solv) {
						approxdif = dist;
						solution = motion;
						break;
					}
					if(dist < approxdif) {
						approxdif = dist;
						approxsol = motion;
					}
				}
			}
		}

		bool solved = false;
		bool approximate = false;
		if(solution == NULL) {
			solution = approxsol;
			approximate = true;
		}

		if(solution != NULL) {
			lastGoalMotion_ = solution;

			/* construct the solution path */
			std::vector<Motion *> mpath;
			while(solution != NULL) {
				mpath.push_back(solution);
				solution = solution->parent;
			}

			/* set the solution path */
			PathControl *path = new PathControl(si_);
			for(int i = mpath.size() - 1 ; i >= 0 ; --i)
				if(mpath[i]->parent)
					path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
				else
					path->append(mpath[i]->state);
			solved = true;
			pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
		}

		if(rmotion->state)
			si_->freeState(rmotion->state);
		if(rmotion->control)
			siC_->freeControl(rmotion->control);
		delete rmotion;
		si_->freeState(xstate);

		OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

		return base::PlannerStatus(solved, approximate);
	}

	virtual void clear() {
		RRT::clear();
		// delete shellsampler_;
		// shellsampler_ = NULL;
	}

protected:

	ompl::base::FBiasedShellStateSampler *shellsampler_;
	const FileMap &params;
	double samplerInitializationTime = 0;
};

}
}
