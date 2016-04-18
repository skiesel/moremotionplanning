#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <ompl/base/GenericParam.h>
#include "ompl/tools/config/SelfConfig.h"

namespace ompl {

namespace control {

class RestartingRRTWithPruning : public ompl::control::RRTLocal {
public:

	/** \brief Constructor */
	RestartingRRTWithPruning(const SpaceInformationPtr &si) : ompl::control::RRTLocal(si) {
		setName("Restarting RRT With Pruning");

		useHeuristic = true;

		propagationStepSize = siC_->getPropagationStepSize();
	}

	virtual ~RestartingRRTWithPruning() {}

	/** \brief Continue solving for some amount of time. Return true if solution was found. */
	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		start = clock();

		ompl::base::PlannerStatus status(false, true);

		optimizationObjective = globalParameters.optimizationObjective;
		optimizationObjective->setCostThreshold(optimizationObjective->infiniteCost());

		while(ptc == false) {

			clear();
			auto newStatus = subSolve(ptc);

			if(newStatus == ompl::base::PlannerStatus::EXACT_SOLUTION) {
				status = newStatus;
			}
		}

		return status;
	}

protected:

	/** \brief Continue solving for some amount of time. Return true if solution was found. */
	base::PlannerStatus subSolve(const base::PlannerTerminationCondition &ptc) {
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

		if(!sampler_)
			sampler_ = si_->allocStateSampler();
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
			else
				sampler_->sampleUniform(rstate);

#ifdef STREAM_GRAPHICS
			streamPoint(rmotion->state, 0, 1, 0, 1);
#endif

			/* find closest state in the tree */
			Motion *nmotion = (Motion*)nn_->nearest(rmotion);

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

#ifdef STREAM_GRAPHICS
						// streamPoint(pstates[p], 1, 0, 0, 1);
#endif

						//we need multiple copies of rctrl
						motion->control = siC_->allocControl();
						siC_->copyControl(motion->control, rctrl);
						motion->steps = 1;
						motion->parent = lastmotion;
						lastmotion = motion;

						motion->updateGValue(propagationStepSize);

						ompl::base::Cost h = useHeuristic ? optimizationObjective->costToGo(motion->state, goal) : ompl::base::Cost(0);
						ompl::base::Cost f = optimizationObjective->combineCosts(motion->g, h);
						if(!optimizationObjective->isSatisfied(f)) {
							break;
						}

						nn_->add(motion);
						double dist = 0.0;
						solved = goal->isSatisfied(motion->state, &dist);
						if(solved && optimizationObjective->isSatisfied(motion->g)) {
							approxdif = dist;
							solution = motion;

							optimizationObjective->setCostThreshold(motion->g);

							globalParameters.solutionStream.addSolution(motion->g, start);

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

					motion->updateGValue(propagationStepSize);

#ifdef STREAM_GRAPHICS
					// streamPoint(nmotion->state, 1, 0, 0, 1);
					// streamPoint(motion->state, 1, 0, 0, 1);
#endif

					ompl::base::Cost h = useHeuristic ? optimizationObjective->costToGo(motion->state, goal) : ompl::base::Cost(0);
					ompl::base::Cost f = optimizationObjective->combineCosts(motion->g, h);
					if(optimizationObjective->isSatisfied(f)) {
						nn_->add(motion);
						double dist = 0.0;
						bool solv = goal->isSatisfied(motion->state, &dist);
						if(solv && optimizationObjective->isSatisfied(motion->g)) {
							approxdif = dist;
							solution = motion;

							optimizationObjective->setCostThreshold(motion->g);

							globalParameters.solutionStream.addSolution(motion->g, start);

							break;
						}
						if(dist < approxdif) {
							approxdif = dist;
							approxsol = motion;
						}
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
#ifdef STREAM_GRAPHICS
				streamPoint(solution->state, 0, 0, 1, 1);
#endif
				mpath.push_back(solution);
				solution = (Motion*)solution->parent;
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

	class Motion : public RRTLocal::Motion {
	public:
		Motion() : RRTLocal::Motion(), g(0.) {}
		Motion(const SpaceInformation *si) : RRTLocal::Motion(si), g(0.) {}
		~Motion() {}

		void updateGValue(double propagationStepSize) {
			g = ompl::base::Cost(((Motion*)parent)->g.value() + propagationStepSize * steps);
		}

		ompl::base::Cost g;
	};

	bool useHeuristic;
	double propagationStepSize;
	ompl::base::State *goalState;
	ompl::base::OptimizationObjectivePtr optimizationObjective;
	clock_t start;
};

}
}