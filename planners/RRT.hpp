/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRT_LOCAL_
#define OMPL_CONTROL_PLANNERS_RRT_RRT_LOCAL_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

namespace ompl {

namespace control {

/**
   @anchor cRRT
   @par Short description
   RRT is a tree-based motion planner that uses the following
   idea: RRT samples a random state @b qr in the state space,
   then finds the state @b qc among the previously seen states
   that is closest to @b qr and expands from @b qc towards @b
   qr, until a state @b qm is reached. @b qm is then added to
   the exploration tree.
   This implementation is intended for systems with differential constraints.
   @par External documentation
   S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol. 20, pp. 378–400, May 2001. DOI: [10.1177/02783640122067453](http://dx.doi.org/10.1177/02783640122067453)<br>
   [[PDF]](http://ijr.sagepub.com/content/20/5/378.full.pdf)
   [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
*/

/** \brief Rapidly-exploring Random Tree */
class RRTLocal : public base::Planner {
public:

	/** \brief Constructor */
	RRTLocal(const SpaceInformationPtr &si) : base::Planner(si, "RRTLocal") {
		specs_.approximateSolutions = true;
		siC_ = si.get();
		addIntermediateStates_ = false;
		lastGoalMotion_ = NULL;

		goalBias_ = 0.05;

		Planner::declareParam<double>("goal_bias", this, &RRTLocal::setGoalBias, &RRTLocal::getGoalBias, "0.:.05:1.");
		Planner::declareParam<bool>("intermediate_states", this, &RRTLocal::setIntermediateStates, &RRTLocal::getIntermediateStates);
	}

	virtual  ~RRTLocal() {
		freeMemory();
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

	/** \brief Clear datastructures. Call this function if the
	    input data to the planner has changed and you do not
	    want to continue planning */
	virtual void clear() {
		Planner::clear();
		sampler_.reset();
		controlSampler_.reset();
		freeMemory();
		if(nn_)
			nn_->clear();
		lastGoalMotion_ = NULL;
	}

	/** In the process of randomly selecting states in the state
	    space to attempt to go towards, the algorithm may in fact
	    choose the actual goal state, if it knows it, with some
	    probability. This probability is a real number between 0.0
	    and 1.0; its value should usually be around 0.05 and
	    should not be too large. It is probably a good idea to use
	    the default value. */
	void setGoalBias(double goalBias) {
		goalBias_ = goalBias;
	}

	/** \brief Get the goal bias the planner is using */
	double getGoalBias() const {
		return goalBias_;
	}

	/** \brief Return true if the intermediate states generated along motions are to be added to the tree itself */
	bool getIntermediateStates() const {
		return addIntermediateStates_;
	}

	/** \brief Specify whether the intermediate states generated along motions are to be added to the tree itself */
	void setIntermediateStates(bool addIntermediateStates) {
		addIntermediateStates_ = addIntermediateStates;
	}

	virtual void getPlannerData(base::PlannerData &data) const {
		Planner::getPlannerData(data);

		std::vector<Motion *> motions;
		if(nn_)
			nn_->list(motions);

		double delta = siC_->getPropagationStepSize();

		if(lastGoalMotion_)
			data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

		for(unsigned int i = 0 ; i < motions.size() ; ++i) {
			const Motion *m = motions[i];
			if(m->parent) {
				if(data.hasControls())
					data.addEdge(base::PlannerDataVertex(m->parent->state),
					             base::PlannerDataVertex(m->state),
					             control::PlannerDataEdgeControl(m->control, m->steps * delta));
				else
					data.addEdge(base::PlannerDataVertex(m->parent->state),
					             base::PlannerDataVertex(m->state));
			} else
				data.addStartVertex(base::PlannerDataVertex(m->state));
		}
	}


	/** \brief Set a different nearest neighbors datastructure */
	template<template<typename T> class NN>
	void setNearestNeighbors() {
		nn_.reset(new NN<Motion *>());
	}

	virtual void setup() {
		base::Planner::setup();
		if(!nn_)
			nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		nn_->setDistanceFunction(boost::bind(&RRTLocal::distanceFunction, this, _1, _2));
	}

protected:


	/** \brief Representation of a motion

	    This only contains pointers to parent motions as we
	    only need to go backwards in the tree. */
	class Motion {
	public:

		Motion() : state(NULL), control(NULL), steps(0), parent(NULL) {
		}

		/** \brief Constructor that allocates memory for the state and the control */
		Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL) {
		}

		virtual ~Motion() {
		}

		/** \brief The state contained by the motion */
		base::State       *state;

		/** \brief The control contained by the motion */
		Control           *control;

		/** \brief The number of steps the control is applied for */
		unsigned int       steps;

		/** \brief The parent motion in the exploration tree */
		Motion            *parent;
	};

	/** \brief Free the memory allocated by this planner */
	void freeMemory() {
		if(nn_) {
			std::vector<Motion *> motions;
			nn_->list(motions);
			for(unsigned int i = 0 ; i < motions.size() ; ++i) {
				if(motions[i]->state)
					si_->freeState(motions[i]->state);
				if(motions[i]->control)
					siC_->freeControl(motions[i]->control);
				delete motions[i];
			}
		}
	}

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion *a, const Motion *b) const {
		return si_->distance(a->state, b->state);
	}

	/** \brief State sampler */
	base::StateSamplerPtr                          sampler_;

	/** \brief Control sampler */
	DirectedControlSamplerPtr                      controlSampler_;

	/** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
	const SpaceInformation                        *siC_;

	/** \brief A nearest-neighbors datastructure containing the tree of motions */
	boost::shared_ptr< NearestNeighbors<Motion *> > nn_;

	/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
	double                                         goalBias_;

	/** \brief Flag indicating whether intermediate states are added to the built tree of motions */
	bool                                           addIntermediateStates_;

	/** \brief The random number generator */
	RNG                                            rng_;

	/** \brief The most recent goal motion.  Used for PlannerData computation */
	Motion                                         *lastGoalMotion_;
};

}
}

#endif
