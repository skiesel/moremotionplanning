/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

#ifndef OMPL_CONTROL_PLANNERS_SST_SST_LOCAL_
#define OMPL_CONTROL_PLANNERS_SST_SST_LOCAL_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl {
namespace control {
/**
   @anchor cSST
   @par Short description
   \ref cSST "SST" (Stable Sparse RRT) is a asymptotically near-optimal incremental
   sampling-based motion planning algorithm for systems with dynamics. It makes use
   of random control inputs to perform a search for the best control inputs to explore
   the state space.
   @par External documentation
   Yanbo Li, Zakary Littlefield, Kostas E. Bekris, Sampling-based
   Asymptotically Optimal Sampling-based Kinodynamic Planning.
   [[PDF]](http://arxiv.org/abs/1407.2896)
*/
class SSTLocal : public base::Planner {
public:

	/** \brief Constructor */
	SSTLocal(const SpaceInformationPtr &si, const FileMap &params) : base::Planner(si, "SST") {
		siC_ = si.get();

		goalBias_ = 0.05; //params.doubleVal("GoalBias");
		selectionRadius_ = params.doubleVal("SelectionRadius");
		pruningRadius_ = params.doubleVal("PruningRadius");

		Planner::declareParam<double>("goal_bias", this, &SSTLocal::setGoalBias, &SSTLocal::getGoalBias, "0.:.05:1.");
		Planner::declareParam<double>("selection_radius", this, &SSTLocal::setSelectionRadius, &SSTLocal::getSelectionRadius, "0.:.1:100");
		Planner::declareParam<double>("pruning_radius", this, &SSTLocal::setPruningRadius, &SSTLocal::getPruningRadius, "0.:.1:100");
	}

	virtual ~SSTLocal() {
		freeMemory();
	}

	virtual void setup() {
		base::Planner::setup();
		if(!nn_)
			nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		nn_->setDistanceFunction(std::bind(&SSTLocal::distanceFunction, this,
		                                   std::placeholders::_1, std::placeholders::_2));
		if(!witnesses_)
			witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		witnesses_->setDistanceFunction(std::bind(&SSTLocal::distanceFunction, this,
		                                std::placeholders::_1, std::placeholders::_2));

		opt_ = globalParameters.optimizationObjective;
		opt_->setCostThreshold(opt_->infiniteCost());
	}

	/** \brief Continue solving for some amount of time. Return true if solution was found. */
	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		start = clock();

		checkValidity();
		base::Goal                   *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

		while(const base::State *st = pis_.nextStart()) {
			Motion *motion = new Motion(siC_);
			si_->copyState(motion->state_, st);
			siC_->nullControl(motion->control_);
			nn_->add(motion);
			motion->accCost_ = opt_->identityCost();
			findClosestWitness(motion);
		}

		if(nn_->size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		if(!sampler_)
			sampler_ = si_->allocStateSampler();
		if(!controlSampler_)
			controlSampler_ = siC_->allocDirectedControlSampler();

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

		Motion *solution  = nullptr;
		Motion *approxsol = nullptr;
		double  approxdif = std::numeric_limits<double>::infinity();
		bool solved = false;

		Motion      *rmotion = new Motion(siC_);
		base::State  *rstate = rmotion->state_;
		Control       *rctrl = rmotion->control_;
		base::State  *xstate = si_->allocState();

		unsigned iterations = 0;

		while(ptc == false) {
			
#ifdef STREAM_GRAPHICS
			// globalIterations++;
			// if(globalIterations % 1000 == 0) {
			// 	dumpCurrentTree();
			// }
#endif

			/* sample random state (with goal biasing) */
			if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
				goal_s->sampleGoal(rstate);
			else
				sampler_->sampleUniform(rstate);

#ifdef STREAM_GRAPHICS
			// streamPoint(rstate, 0, 1, 0, 1);
#endif

			/* find closest state in the tree */
			Motion *nmotion = selectNode(rmotion);

			unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control_, nmotion->state_, rmotion->state_);

			if(cd >= siC_->getMinControlDuration()) {
				base::Cost incCost(cd * siC_->getPropagationStepSize());
				base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);

				double dist = 0.0;
				bool solv = goal->isSatisfied(rmotion->state_, &dist);
				if(solv && opt_->isSatisfied(cost)) {
					opt_->setCostThreshold(cost);

					globalParameters.solutionStream.addSolution(cost, start);

					OMPL_INFORM("Found solution with cost %.2f", cost.value());

					solved = true;
				}

				Witness *closestWitness = findClosestWitness(rmotion);

				if(closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost,closestWitness->rep_->accCost_)) {
					Motion *oldRep = closestWitness->rep_;
					/* create a motion */
					Motion *motion = new Motion(siC_);
					motion->accCost_ = cost;
					si_->copyState(motion->state_, rmotion->state_);
					siC_->copyControl(motion->control_, rctrl);
					motion->steps_ = cd;
					motion->parent_ = nmotion;
					nmotion->numChildren_++;
					closestWitness->linkRep(motion);

#ifdef STREAM_GRAPHICS
					streamPoint(motion->state_, 1, 0, 0, 1);
#endif

					nn_->add(motion);

					if(oldRep != rmotion) {
						oldRep->inactive_ = true;
						nn_->remove(oldRep);
						while(oldRep->inactive_ && oldRep->numChildren_==0) {
							if(oldRep->state_)
								si_->freeState(oldRep->state_);
							if(oldRep->control_)
								siC_->freeControl(oldRep->control_);

							oldRep->state_=nullptr;
							oldRep->control_=nullptr;
							oldRep->parent_->numChildren_--;
							Motion *oldRepParent = oldRep->parent_;
							delete oldRep;
							oldRep = oldRepParent;
						}
					}

				}
			}
			iterations++;
		}

		si_->freeState(xstate);
		if(rmotion->state_)
			si_->freeState(rmotion->state_);
		if(rmotion->control_)
			siC_->freeControl(rmotion->control_);
		delete rmotion;

		OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(),iterations);

		return base::PlannerStatus(solved, false);
	}


	virtual void getPlannerData(base::PlannerData &data) const {
		Planner::getPlannerData(data);
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
		if(witnesses_)
			witnesses_->clear();
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

	/**
	    \brief Set the radius for selecting nodes relative to random sample.

	    This radius is used to mimic behavior of RRT* in that it promotes
	    extending from nodes with good path cost from the root of the tree.
	    Making this radius larger will provide higher quality paths, but has two
	    major drawbacks; exploration will occur much more slowly and exploration
	    around the boundary of the state space may become impossible. */
	void setSelectionRadius(double selectionRadius) {
		selectionRadius_  = selectionRadius;
	}

	/** \brief Get the selection radius the planner is using */
	double getSelectionRadius() const {
		return selectionRadius_;
	}


	/**
	    \brief Set the radius for pruning nodes.

	    This is the radius used to surround nodes in the witness set.
	    Within this radius around a state in the witness set, only one
	    active tree node can exist. This limits the size of the tree and
	    forces computation to focus on low path costs nodes. If this value
	    is too large, narrow passages will be impossible to traverse. In addition,
	    children nodes may be removed if they are not at least this distance away
	    from their parent nodes.*/
	void setPruningRadius(double pruningRadius) {
		pruningRadius_  = pruningRadius;
	}

	/** \brief Get the pruning radius the planner is using */
	double getPruningRadius() const {
		return pruningRadius_;
	}

	/** \brief Set a different nearest neighbors datastructure */
	template<template<typename T> class NN>
	void setNearestNeighbors() {
		nn_.reset(new NN<Motion *>());
		witnesses_.reset(new NN<Motion *>());
	}

protected:


	/** \brief Representation of a motion

	    This only contains pointers to parent motions as we
	    only need to go backwards in the tree. */
	class Motion {
	public:

		Motion() : accCost_(0), state_(nullptr), control_(nullptr), steps_(0), parent_(nullptr), numChildren_(0), inactive_(false) {
		}

		/** \brief Constructor that allocates memory for the state and the control */
		Motion(const SpaceInformation *si) : accCost_(0), state_(si->allocState()), control_(si->allocControl()), steps_(0), parent_(nullptr), numChildren_(0), inactive_(false) {
		}

		virtual ~Motion() {
		}

		virtual base::State *getState() const {
			return state_;
		}
		virtual Motion *getParent() const {
			return parent_;
		}

		base::Cost accCost_;

		/** \brief The state contained by the motion */
		base::State       *state_;

		/** \brief The control contained by the motion */
		Control           *control_;

		/** \brief The number of steps_ the control is applied for */
		unsigned int       steps_;

		/** \brief The parent motion in the exploration tree */
		Motion            *parent_;

		/** \brief Number of children */
		unsigned numChildren_;

		/** \brief If inactive, this node is not considered for selection.*/
		bool inactive_;


	};

	class Witness : public Motion {
	public:

		Witness() : Motion(), rep_(nullptr) {
		}

		Witness(const SpaceInformation *si) : Motion(si), rep_(nullptr) {
		}
		virtual base::State *getState() const {
			return rep_->state_;
		}
		virtual Motion *getParent() const {
			return rep_->parent_;
		}

		void linkRep(Motion *lRep) {
			rep_ = lRep;
		}

		/** \brief The node in the tree that is within the pruning radius.*/
		Motion *rep_;
	};

	/** \brief Finds the best node in the tree withing the selection radius around a random sample.*/
	Motion *selectNode(Motion *sample) {
		std::vector<Motion *> ret;
		Motion *selected = nullptr;
		base::Cost bestCost = opt_->infiniteCost();
		nn_->nearestR(sample, selectionRadius_, ret);
		for(unsigned int i = 0; i < ret.size(); i++) {
			if(!ret[i]->inactive_ && opt_->isCostBetterThan(ret[i]->accCost_, bestCost)) {
				bestCost = ret[i]->accCost_;
				selected = ret[i];
			}
		}
		if(selected == nullptr) {
			int k = 1;
			while(selected == nullptr) {
				nn_->nearestK(sample, k, ret);
				for(unsigned int i=0; i < ret.size() && selected == nullptr; i++)
					if(!ret[i]->inactive_)
						selected = ret[i];
				k += 5;
			}
		}
		return selected;
	}

	/** \brief Find the closest witness node to a newly generated potential node.*/
	Witness *findClosestWitness(Motion *node) {
		if(witnesses_->size() > 0) {
			Witness *closest = static_cast<Witness *>(witnesses_->nearest(node));
			if(distanceFunction(closest,node) > pruningRadius_) {
				closest = new Witness(siC_);
				closest->linkRep(node);
				si_->copyState(closest->state_, node->state_);
				witnesses_->add(closest);
			}
			return closest;
		} else {
			Witness *closest = new Witness(siC_);
			closest->linkRep(node);
			si_->copyState(closest->state_, node->state_);
			witnesses_->add(closest);
			return closest;
		}
	}


	/** \brief Free the memory allocated by this planner */
	void freeMemory() {
		if(nn_) {
			std::vector<Motion *> motions;
			nn_->list(motions);
			for(unsigned int i = 0 ; i < motions.size() ; ++i) {
				if(motions[i]->state_)
					si_->freeState(motions[i]->state_);
				if(motions[i]->control_)
					siC_->freeControl(motions[i]->control_);
				delete motions[i];
			}
		}
		if(witnesses_) {
			std::vector<Motion *> witnesses;
			witnesses_->list(witnesses);
			for(unsigned int i = 0 ; i < witnesses.size() ; ++i) {
				delete witnesses[i];
			}
		}
	}

#ifdef STREAM_GRAPHICS
	void dumpCurrentTree() const {
		streamClearScreen();
		std::vector<Motion *> motions;
		nn_->list(motions);
		for(unsigned int i = 0 ; i < motions.size() ; ++i) {
			if(!motions[i]->inactive_ && motions[i]->parent_ != nullptr) {
				streamLine(motions[i]->parent_->state_, motions[i]->state_, 1,1,1,1);
			}
		}
	}
#endif

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion *a, const Motion *b) const {
		return si_->distance(a->state_, b->state_);
	}

	/** \brief State sampler */
	base::StateSamplerPtr                          sampler_;

	/** \brief Control sampler */
	DirectedControlSamplerPtr                      controlSampler_;

	/** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
	const SpaceInformation                        *siC_;

	/** \brief A nearest-neighbors datastructure containing the tree of motions */
	std::shared_ptr< NearestNeighbors<Motion *> > nn_;


	/** \brief A nearest-neighbors datastructure containing the tree of witness motions */
	std::shared_ptr< NearestNeighbors<Motion *> > witnesses_;

	/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
	double                                         goalBias_;

	/** \brief The radius for determining the node selected for extension. */
	double                                         selectionRadius_;

	/** \brief The radius for determining the size of the pruning region. */
	double                                         pruningRadius_;

	/** \brief The random number generator */
	RNG                                            rng_;


	/** \brief The optimization objective. */
	base::OptimizationObjectivePtr                 opt_;

	clock_t                                        start;

	unsigned int                                   globalIterations = 0;

};
}
}

#endif