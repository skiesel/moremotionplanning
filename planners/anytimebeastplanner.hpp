#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "../structs/filemap.hpp"
#include "../samplers/anytimebeastsampler.hpp"

#include "../samplers/beastsampler_dstar.hpp"

namespace ompl {

namespace control {

class AnytimeBeastPlanner : public ompl::control::RRT {
protected:
	class Witness;

public:
	
	/** \brief Constructor */
	AnytimeBeastPlanner(const SpaceInformationPtr &si, const FileMap &params) :
		ompl::control::RRT(si), params(params) {

		setName("AnytimeBeastPlanner");

		propagationStepSize = siC_->getPropagationStepSize();

		selectionRadius = params.doubleVal("SelectionRadius");
		pruningRadius = params.doubleVal("PruningRadius");
		n0 = params.doubleVal("N0");
		xi = params.doubleVal("Xi");

		Planner::declareParam<bool>("intermediate_states", this, &AnytimeBeastPlanner::setIntermediateStates, &AnytimeBeastPlanner::getIntermediateStates);

		Planner::declareParam<double>("stateradius", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getStateRadius);
		Planner::declareParam<unsigned int>("prmsize", this, &AnytimeBeastPlanner::ignoreSetterUnsigedInt, &AnytimeBeastPlanner::getPRMSize);
		Planner::declareParam<unsigned int>("numprmedges", this, &AnytimeBeastPlanner::ignoreSetterUnsigedInt, &AnytimeBeastPlanner::getNumPRMEdges);

		Planner::declareParam<double>("validedgedistributionalpha", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getValidEdgeDistributionAlpha);
		Planner::declareParam<double>("validedgedistributionbeta", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getValidEdgeDistributionBeta);
		Planner::declareParam<double>("invalidedgedistributionalpha", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getInvalidEdgeDistributionAlpha);
		Planner::declareParam<double>("invalidedgedistributionbeta", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getInvalidEdgeDistributionBeta);

		Planner::declareParam<double>("selectionradius", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getSelectionRadius);
		Planner::declareParam<double>("pruningradius", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getPruningRadius);
		Planner::declareParam<double>("xi", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getXi);
		Planner::declareParam<double>("n0", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getN0);

		//Obviously this isn't really a parameter but I have no idea how else to get it into the output file through the benchmarker
		Planner::declareParam<double>("samplerinitializationtime", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getSamplerInitializationTime);
	}

	virtual ~AnytimeBeastPlanner() {
		freeMemory();
	}

	void ignoreSetterDouble(double) const {}
	void ignoreSetterUnsigedInt(unsigned int) const {}
	void ignoreSetterBool(bool) const {}

	double getStateRadius() const { return params.doubleVal("StateRadius"); }
	unsigned int getPRMSize() const { return params.integerVal("PRMSize"); }
	unsigned int getNumPRMEdges() const { return params.integerVal("NumEdges"); }
	double getValidEdgeDistributionAlpha() const { return params.doubleVal("ValidEdgeDistributionAlpha"); }
	double getValidEdgeDistributionBeta() const { return params.doubleVal("ValidEdgeDistributionBeta"); }
	double getInvalidEdgeDistributionAlpha() const { return params.doubleVal("InvalidEdgeDistributionAlpha"); }
	double getInvalidEdgeDistributionBeta() const { return params.doubleVal("InvalidEdgeDistributionBeta"); }

	double getSelectionRadius() const { return params.doubleVal("SelectionRadius"); }
	double getPruningRadius() const { return params.doubleVal("PruningRadius"); }
	double getXi() const { return params.doubleVal("Xi"); }
	unsigned int getN0() const { return params.doubleVal("N0"); }
	
	double getSamplerInitializationTime() const { return samplerInitializationTime; }

	bool getIntermediateStates() const { return addIntermediateStates_; }
	void setIntermediateStates(bool add) { addIntermediateStates_ = add; }

	virtual void setup() {
		ompl::control::RRT::setup();
		if(!witnesses)
			witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		witnesses->setDistanceFunction(boost::bind(&AnytimeBeastPlanner::distanceFunction, this, _1, _2));
	}

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		start = clock();

		checkValidity();
		base::Goal                   *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

		while(const base::State *st = pis_.nextStart()) {
			Motion *motion = new MotionWithCost(siC_);
			si_->copyState(motion->state, st);
			siC_->nullControl(motion->control);
			nn_->add(motion);

			auto witness = new Witness(siC_);
			si_->copyState(witness->state, motion->state);
			witness->rep = motion;
			witnesses->add(witness);
		}

		if(nn_->size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		optimizationObjective = globalParameters.optimizationObjective;
		optimizationObjective->setCostThreshold(optimizationObjective->infiniteCost());

		unsigned int iterations = 0;
		double j = 0;
		double d = siC_->getStateSpace()->getDimension();
		double l = siC_->getControlSpace()->getDimension();
		double SSTStarIteration = 0;
		unsigned int iterationBound = n0;

		if(!newsampler) {
			auto start = clock();

			newsampler = new ompl::base::AnytimeBeastSampler((ompl::base::SpaceInformation *)siC_, pdef_->getStartState(0), pdef_->getGoal(), goal_s, optimizationObjective, params);

			newsampler->initialize();

			samplerInitializationTime = (double)(clock() - start) / CLOCKS_PER_SEC;
		}
		if(!controlSampler_)
			controlSampler_ = siC_->allocDirectedControlSampler();

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

		while(ptc == false) {
			subSolve(ptc, goal, iterationBound);

			SSTStarIteration++;
			selectionRadius *= xi;
			pruningRadius *= xi;
			iterationBound += (1 + log(SSTStarIteration)) * pow(xi, -(d + l + 1) * SSTStarIteration) * n0;
		}

		return ompl::base::PlannerStatus(false, false);
	}

protected:

	void subSolve(const base::PlannerTerminationCondition &ptc, base::Goal* goal, unsigned int iterationBound) {
		Motion      *rmotion = new MotionWithCost(siC_);
		base::State  *rstate = rmotion->state;
		Control       *rctrl = rmotion->control;

		Motion *resusableMotion = new MotionWithCost(siC_);

		unsigned int iterations = 0;

		while(ptc == false && iterations < iterationBound) {
			iterations++;

			Motion *nmotion = NULL;

			newsampler->sample(resusableMotion->state, rstate);

			// nmotion = nn_->nearest(resusableMotion);
			// nmotion = nn_->nearest(rmotion);
			nmotion = selectNode(rmotion);

			unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

			if(addIntermediateStates_) {
				std::vector<base::State *> pstates;
				cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

				if(cd >= siC_->getMinControlDuration()) {
					Motion *lastmotion = nmotion;
					bool solved = false;
					size_t p = 0;
					std::vector<Motion*> addedMotions;
					for(; p < pstates.size(); ++p) {
						Motion *motion = new MotionWithCost();
						motion->state = pstates[p];

						motion->control = siC_->allocControl();
						siC_->copyControl(motion->control, rctrl);
						motion->steps = 1;
						motion->parent = lastmotion;
						((MotionWithCost*)motion)->updateGValue(propagationStepSize);

						ompl::base::Cost h = optimizationObjective->costToGo(motion->state, goal);
						ompl::base::Cost f = optimizationObjective->combineCosts(((MotionWithCost*)motion)->g, h);
						if(!optimizationObjective->isSatisfied(f)) {
							break;
						}

						newsampler->reached(lastmotion->state, ((MotionWithCost*)lastmotion)->g.value(), pstates[p], ((MotionWithCost*)motion)->g.value());

						((MotionWithCost*)lastmotion)->numChildren++;
						addedMotions.emplace_back(motion);
						nn_->add(motion);

						lastmotion = motion;

						double dist = 0.0;
						solved = goal->isSatisfied(motion->state, &dist);
						if(solved && optimizationObjective->isSatisfied(((MotionWithCost*)motion)->g)) {
							globalParameters.solutionStream.addSolution(((MotionWithCost*)motion)->g, start);
							newsampler->foundSolution(((MotionWithCost*)motion)->g);
							optimizationObjective->setCostThreshold(((MotionWithCost*)motion)->g);
							break;
						}
					}

					// post process the branch just added for SST* like pruning from leaf to root
					for (auto addedMotionsIterator = addedMotions.rbegin(); addedMotionsIterator != addedMotions.rend(); ++addedMotionsIterator) {
						Motion *motion = *addedMotionsIterator;
						Witness *closestWitness = findClosestWitness(motion);
						if(closestWitness->rep == motion ||
							optimizationObjective->isCostBetterThan(((MotionWithCost*)motion)->g, ((MotionWithCost*)closestWitness->rep)->g)) {

							Motion *oldRep = closestWitness->rep;
							closestWitness->linkRep(motion);
							if(oldRep != motion) {
								cleanupTree(oldRep);
							}

						} else {
							cleanupTree(motion);
						}
					}

					//free any states after we hit the goal
					while(++p < pstates.size())
						si_->freeState(pstates[p]);
					if(solved)
						break;
				} else {
					for(size_t p = 0 ; p < pstates.size(); ++p)
						si_->freeState(pstates[p]);
				}
			} else {
				if(cd >= siC_->getMinControlDuration()) {
					/* create a motion */
					Motion *motion = new MotionWithCost(siC_);

					si_->copyState(motion->state, rmotion->state);
					siC_->copyControl(motion->control, rctrl);
					motion->steps = cd;
					motion->parent = nmotion;
					((MotionWithCost*)motion)->updateGValue(propagationStepSize);

					ompl::base::Cost h = optimizationObjective->costToGo(motion->state, goal);
					ompl::base::Cost f = optimizationObjective->combineCosts(((MotionWithCost*)motion)->g, h);
					if(optimizationObjective->isSatisfied(f)) {

						double dist = 0.0;
						bool solv = goal->isSatisfied(motion->state, &dist);
						if(solv && optimizationObjective->isSatisfied(((MotionWithCost*)motion)->g)) {
							globalParameters.solutionStream.addSolution(((MotionWithCost*)motion)->g, start);
							newsampler->foundSolution(((MotionWithCost*)motion)->g);
							optimizationObjective->setCostThreshold(((MotionWithCost*)motion)->g);
							break;
						}

						Witness *closestWitness = findClosestWitness(motion);

						if(closestWitness->rep == motion || optimizationObjective->isCostBetterThan(((MotionWithCost*)nmotion)->g, ((MotionWithCost*)closestWitness->rep)->g)) {

							Motion *oldRep = closestWitness->rep;
							closestWitness->linkRep(motion);
							((MotionWithCost*)nmotion)->numChildren++;

							newsampler->reached(nmotion->state, ((MotionWithCost*)nmotion)->g.value(), motion->state, ((MotionWithCost*)motion)->g.value());

							((MotionWithCost*)nmotion)->numChildren++;
							nn_->add(motion);

							if(oldRep != motion) {
								cleanupTree(oldRep);
							}
						}
					}
				}
			}
		}

		if(rmotion->state)
			si_->freeState(rmotion->state);
		if(rmotion->control)
			siC_->freeControl(rmotion->control);
		delete rmotion;

		OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
	}

	double distanceFunction(const Motion *a, const Motion *b) const {
		return si_->distance(a->state, b->state);
	}

	Motion *selectNode(Motion *sample) {
		std::vector<Motion *> ret;
		Motion *selected = nullptr;
		base::Cost bestCost = optimizationObjective->infiniteCost();
		nn_->nearestR(sample, selectionRadius, ret);
		for(unsigned int i = 0; i < ret.size(); i++) {
			if(!((MotionWithCost*)ret[i])->inactive && optimizationObjective->isCostBetterThan(((MotionWithCost*)ret[i])->g, bestCost)) {
				bestCost = ((MotionWithCost*)ret[i])->g;
				selected = ret[i];
			}
		}
		if(selected == nullptr) {
			int k = 1;
			while(selected == nullptr) {
				nn_->nearestK(sample, k, ret);
				for(unsigned int i=0; i < ret.size() && selected == nullptr; i++)
					if(!((MotionWithCost*)ret[i])->inactive)
						selected = ret[i];
				k += 5;
			}
		}
		return selected;
	}

	Witness *findClosestWitness(Motion *node) {
		if(witnesses->size() > 0) {
			Witness *closest = (Witness *)witnesses->nearest(node);
			if(distanceFunction(closest,node) > pruningRadius) {
				closest = new Witness(siC_);
				closest->linkRep(node);
				si_->copyState(closest->state, node->state);
				witnesses->add(closest);
			}
			return closest;
		} else {
			Witness *closest = new Witness(siC_);
			closest->linkRep(node);
			si_->copyState(closest->state, node->state);
			witnesses->add(closest);
			return closest;
		}
	}

	void cleanupTree(Motion *oldRep) {
		((MotionWithCost*)oldRep)->inactive = true;

		nn_->remove(oldRep);
		while(((MotionWithCost*)oldRep)->inactive && ((MotionWithCost*)oldRep)->numChildren == 0) {
			newsampler->remove(oldRep->state);

			if(oldRep->state)
				si_->freeState(oldRep->state);
			if(oldRep->control)
				siC_->freeControl(oldRep->control);

			oldRep->state = nullptr;
			oldRep->control = nullptr;
			((MotionWithCost*)oldRep->parent)->numChildren--;
			Motion *oldRepParent = oldRep->parent;
			delete oldRep;
			oldRep = oldRepParent;
		}
	}

	class MotionWithCost : public Motion {
	public:
		MotionWithCost() {}

		MotionWithCost(const SpaceInformation *si) : Motion(si) {}

		void updateGValue(double propagationStepSize) {
			g = ompl::base::Cost(((MotionWithCost*)parent)->g.value() + propagationStepSize * steps);
		}

		ompl::base::Cost g = ompl::base::Cost(0);
		unsigned int numChildren = 0;
		bool inactive = false;
	};

	class Witness : public Motion {
	public:

		Witness() : Motion(), rep(nullptr) {}

		Witness(const SpaceInformation *si) : Motion(si), rep(nullptr) {}

		virtual base::State *getState() const {
			return rep->state;
		}

		virtual Motion *getParent() const {
			return rep->parent;
		}

		void linkRep(Motion *lRep) {
			rep = lRep;
		}

		Motion *rep;
	};

	ompl::base::AnytimeBeastSampler *newsampler = NULL;

	std::shared_ptr< NearestNeighbors<Motion *> > witnesses;
	base::OptimizationObjectivePtr optimizationObjective;
	double propagationStepSize, selectionRadius, pruningRadius, xi, n0, samplerInitializationTime = 0;
	clock_t start;
	unsigned int globalIterations = 0;

	const FileMap &params;

};

}

}
