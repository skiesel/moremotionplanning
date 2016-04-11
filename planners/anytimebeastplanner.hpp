#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "../structs/filemap.hpp"
#include "../samplers/anytimebeastsampler.hpp"

namespace ompl {

namespace control {

class AnytimeBeastPlanner : public ompl::base::Planner {
protected:
	class Witness;

public:
	
	/** \brief Constructor */
	AnytimeBeastPlanner(const SpaceInformationPtr &si, const FileMap &params) :
		base::Planner(si, "AnytimeBeastPlanner"), newsampler(NULL), params(params) {

		siC = si.get();
		propagationStepSize = siC->getPropagationStepSize();
		useHeuristic = true;

		selectionRadius = params.doubleVal("SelectionRadius");
		pruningRadius = params.doubleVal("PruningRadius");
		n0 = params.doubleVal("N0");
		xi = params.doubleVal("Xi");

		addIntermediateStates = false;

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

	bool getIntermediateStates() const { return addIntermediateStates; }
	void setIntermediateStates(bool add) { addIntermediateStates = add; }

	virtual void setup() {
		base::Planner::setup();
		if(!nn)
			nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		nn->setDistanceFunction(std::bind(&AnytimeBeastPlanner::distanceFunction, this,
		                                   std::placeholders::_1, std::placeholders::_2));
		if(!witnesses)
			witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
		witnesses->setDistanceFunction(std::bind(&AnytimeBeastPlanner::distanceFunction, this,
		                                std::placeholders::_1, std::placeholders::_2));

		optimizationObjective = pdef_->getOptimizationObjective();
		optimizationObjective->setCostThreshold(optimizationObjective->infiniteCost());
	}

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		start = clock();

		ompl::base::PlannerStatus status(false, true);

		double j = 0;
		double d = siC->getStateSpace()->getDimension();
		double l = siC->getControlSpace()->getDimension();
		double SSTStarIteration = 0;
		unsigned int iterationBound = n0;

		optimizationObjective = pdef_->getOptimizationObjective();
		optimizationObjective->setCostThreshold(optimizationObjective->infiniteCost());

		checkValidity();
		base::Goal                   *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goals = dynamic_cast<base::GoalSampleableRegion *>(goal);

		while(const base::State *st = pis_.nextStart()) {
			Motion *motion = new Motion(siC);
			si_->copyState(motion->state, st);
			siC->nullControl(motion->control);
			nn->add(motion);

			auto witness = new Witness(siC);
			si_->copyState(witness->state, motion->state);
			witness->rep = motion;
			witnesses->add(witness);
		}

		if(nn->size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		if(!newsampler) {
			auto initStart = clock();

			newsampler = new ompl::base::AnytimeBeastSampler((ompl::base::SpaceInformation*)siC, pdef_->getStartState(0), pdef_->getGoal(),
			        goals, optimizationObjective, params);
		
			newsampler->initialize();

			samplerInitializationTime = (double)(clock() - initStart) / CLOCKS_PER_SEC;
		}
		if(!controlSampler)
			controlSampler = siC->allocDirectedControlSampler();

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn->size());

		while(ptc == false) {

			// clear();
			auto newStatus = subSolve(ptc, goal, iterationBound);

			SSTStarIteration++;
			selectionRadius *= xi;
			pruningRadius *= xi;
			iterationBound += (1 + log(SSTStarIteration)) * pow(xi, -(d + l + 1) * SSTStarIteration) * n0;

			if(newStatus == ompl::base::PlannerStatus::EXACT_SOLUTION) {
				status = newStatus;
			}
		}

		return status;
	}

	virtual void clear() {
		Planner::clear();
		// sampler_.reset();
		// controlSampler_.reset();
		// freeMemory();
		// if(nn_)
		// 	nn_->clear();
		// if(witnesses_)
		// 	witnesses_->clear();
	}

protected:

base::PlannerStatus subSolve(const base::PlannerTerminationCondition &ptc, ompl::base::Goal *goal, unsigned int iterationBound) {
		Motion *solution  = NULL;
		Motion *approxsol = NULL;
		double  approxdif = std::numeric_limits<double>::infinity();

		Motion      *rmotion = new Motion(siC);
		base::State  *rstate = rmotion->state;
		Control       *rctrl = rmotion->control;
		base::State  *xstate = si_->allocState();

		Motion *resusableMotion = new Motion(siC);

		unsigned int iterations = 0;

		while(ptc == false && iterations < iterationBound) {
			globalIterations++;
#ifdef STREAM_GRAPHICS
			if(globalIterations % 1000 == 0) {
				dumpCurrentTree();
			}
#endif

			newsampler->sample(resusableMotion->state, rstate);
			Motion *nmotion = selectNode(rmotion);

#ifdef STREAM_GRAPHICS
			streamPoint(rmotion->state, 0, 1, 0, 1);
#endif

			/* sample a random control that attempts to go towards the random state, and also sample a control duration */
			unsigned int cd = controlSampler->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

			if(addIntermediateStates) {
				// this code is contributed by Jennifer Barry
				std::vector<base::State *> pstates;
				cd = siC->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

				if(cd >= siC->getMinControlDuration()) {
					Motion *lastmotion = nmotion;
					bool solved = false;
					size_t p = 0;
					std::vector<Motion*> addedMotions;
					for(; p < pstates.size(); ++p) {
						/* create a motion */
						Motion *motion = new Motion(siC);
						motion->state = pstates[p];

						//we need multiple copies of rctrl
						motion->control = siC->allocControl();
						siC->copyControl(motion->control, rctrl);
						motion->steps = 1;
						motion->parent = lastmotion;
						motion->updateGValue(propagationStepSize);
						lastmotion = motion;

						ompl::base::Cost h = useHeuristic ? optimizationObjective->costToGo(motion->state, goal) : ompl::base::Cost(0);
						ompl::base::Cost f = optimizationObjective->combineCosts(motion->g, h);
						if(!optimizationObjective->isSatisfied(f)) {
							break;
						}

						lastmotion->numChildren++;

						addedMotions.emplace_back(motion);
						newsampler->reached(nmotion->state, nmotion->g.value(), pstates[p], motion->g.value());

#ifdef STREAM_GRAPHICS
						// streamPoint(pstates[p], 1, 0, 0, 1);
#endif

						nn->add(motion);
						double dist = 0.0;
						solved = goal->isSatisfied(motion->state, &dist);
						if(solved && optimizationObjective->isSatisfied(motion->g)) {
							approxdif = dist;
							solution = motion;

							optimizationObjective->setCostThreshold(solution->g);
							globalParameters.solutionStream.addSolution(solution->g, start);
							newsampler->foundSolution(solution->g);
							OMPL_INFORM("Found solution with cost %.2f", solution->g.value());

							break;
						}
						if(dist < approxdif) {
							approxdif = dist;
							approxsol = motion;
						}
					}

					//post process the branch just added for SST* like pruning from leaf to root
					for (auto addedMotionsIterator = addedMotions.rbegin(); addedMotionsIterator != addedMotions.rend(); ++addedMotionsIterator) {
						Motion *motion = *addedMotionsIterator;
						Witness *closestWitness = findClosestWitness(motion);
						if(closestWitness->rep == motion || optimizationObjective->isCostBetterThan(motion->g, closestWitness->rep->g)) {
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
					while(++p < pstates.size()) {
						newsampler->remove(pstates[p]);
						si_->freeState(pstates[p]);
					}
					if(solved)
						break;
				} else
					for(size_t p = 0 ; p < pstates.size(); ++p) {
						newsampler->remove(pstates[p]);
						si_->freeState(pstates[p]);
					}
			} else {
				if(cd >= siC->getMinControlDuration()) {
					/* create a motion */
					Motion *motion = new Motion(siC);

					si_->copyState(motion->state, rmotion->state);
					siC->copyControl(motion->control, rctrl);
					motion->steps = cd;
					motion->parent = nmotion;
					motion->updateGValue(propagationStepSize);

					ompl::base::Cost h = useHeuristic ? optimizationObjective->costToGo(motion->state, goal) : ompl::base::Cost(0);
					ompl::base::Cost f = optimizationObjective->combineCosts(motion->g, h);
					if(optimizationObjective->isSatisfied(f)) {

						Witness *closestWitness = findClosestWitness(rmotion);

						double dist = 0.0;
						bool solv = goal->isSatisfied(motion->state, &dist);
						if(solv && optimizationObjective->isSatisfied(motion->g)) {
							approxdif = dist;
							solution = motion;

							optimizationObjective->setCostThreshold(solution->g);
							globalParameters.solutionStream.addSolution(solution->g, start);
							newsampler->foundSolution(solution->g);
							OMPL_INFORM("Found solution with cost %.2f", solution->g.value());

							break;
						}
						if(dist < approxdif) {
							approxdif = dist;
							approxsol = motion;
						}

						if(closestWitness->rep == rmotion || optimizationObjective->isCostBetterThan(motion->g, closestWitness->rep->g)) {

							Motion *oldRep = closestWitness->rep;
							nmotion->numChildren++;
							closestWitness->linkRep(motion);

							newsampler->reached(nmotion->state, nmotion->g.value(), motion->state, motion->g.value());

#ifdef STREAM_GRAPHICS
							streamPoint(nmotion->state, 1, 0, 0, 1);
							streamPoint(motion->state, 1, 0, 0, 1);
#endif

							nn->add(motion);

							if(oldRep != rmotion) {
								cleanupTree(oldRep);
							}
						}
					}
				}
			}
		}

		bool solved = false;
		bool approximate = false;
		if(solution == nullptr) {
			solution = approxsol;
			approximate = true;
		}

		if(rmotion->state)
			si_->freeState(rmotion->state);
		if(rmotion->control)
			siC->freeControl(rmotion->control);
		delete rmotion;
		si_->freeState(xstate);

		OMPL_INFORM("%s: Created %u states", getName().c_str(), nn->size());

		return base::PlannerStatus(solved, approximate);
	}

	class Motion {
	public:

		Motion() :state(nullptr), control(nullptr) {}

		Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {}

		virtual ~Motion() {}

		virtual base::State *getState() const {
			return state;
		}

		virtual Motion *getParent() const {
			return parent;
		}

		void updateGValue(double propagationStepSize) {
			g = ompl::base::Cost(parent->g.value() + propagationStepSize * steps);
		}

		base::State *state;
		Control *control;
		Motion *parent = nullptr;
		unsigned int steps = 0, numChildren = 0;
		bool inactive = false;
		ompl::base::Cost g = ompl::base::Cost(0);
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

	Motion *selectNode(Motion *sample) {
		std::vector<Motion *> ret;
		Motion *selected = nullptr;
		base::Cost bestCost = optimizationObjective->infiniteCost();
		nn->nearestR(sample, selectionRadius, ret);
		for(unsigned int i = 0; i < ret.size(); i++) {
			if(!ret[i]->inactive && optimizationObjective->isCostBetterThan(ret[i]->g, bestCost)) {
				bestCost = ret[i]->g;
				selected = ret[i];
			}
		}
		if(selected == nullptr) {
			int k = 1;
			while(selected == nullptr) {
				nn->nearestK(sample, k, ret);
				for(unsigned int i=0; i < ret.size() && selected == nullptr; i++)
					if(!ret[i]->inactive)
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
				closest = new Witness(siC);
				closest->linkRep(node);
				si_->copyState(closest->state, node->state);
				witnesses->add(closest);
			}
			return closest;
		} else {
			Witness *closest = new Witness(siC);
			closest->linkRep(node);
			si_->copyState(closest->state, node->state);
			witnesses->add(closest);
			return closest;
		}
	}

	void cleanupTree(Motion *oldRep) {
		oldRep->inactive = true;

		newsampler->remove(oldRep->state);

		nn->remove(oldRep);
		while(oldRep->inactive && oldRep->numChildren == 0) {
			if(oldRep->state)
				si_->freeState(oldRep->state);
			if(oldRep->control)
				siC->freeControl(oldRep->control);

			oldRep->state = nullptr;
			oldRep->control = nullptr;
			oldRep->parent->numChildren--;
			Motion *oldRepParent = oldRep->parent;
			delete oldRep;
			oldRep = oldRepParent;
		}
	}

	void freeMemory() {
		assert(false);
		if(nn) {
			std::vector<Motion *> motions;
			nn->list(motions);
			for(unsigned int i = 0 ; i < motions.size() ; ++i) {
				if(motions[i]->state)
					si_->freeState(motions[i]->state);
				if(motions[i]->control)
					siC->freeControl(motions[i]->control);
				delete motions[i];
			}
		}
		if(witnesses) {
			std::vector<Motion *> witnesseList;
			witnesses->list(witnesseList);
			for(unsigned int i = 0 ; i < witnesseList.size() ; ++i) {
				delete witnesseList[i];
			}
		}
	}

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion *a, const Motion *b) const {
		return si_->distance(a->state, b->state);
	}

#ifdef STREAM_GRAPHICS
	void dumpCurrentTree() const {
		streamClearScreen();
		std::vector<Motion *> motions;
		nn->list(motions);
		for(unsigned int i = 0 ; i < motions.size() ; ++i) {
			if(!motions[i]->inactive && motions[i]->parent != nullptr) {
				streamLine(motions[i]->parent->state, motions[i]->state, 1,1,1,1);
			}
		}
	}
#endif

	ompl::base::AnytimeBeastSampler *newsampler;
	DirectedControlSamplerPtr controlSampler;
	const SpaceInformation *siC;
	std::shared_ptr< NearestNeighbors<Motion *> > nn, witnesses;
	RNG rng;
	base::OptimizationObjectivePtr optimizationObjective;
	double propagationStepSize, selectionRadius, pruningRadius, xi, n0, samplerInitializationTime = 0;
	clock_t start;
	bool addIntermediateStates, useHeuristic;
	unsigned int globalIterations = 0;

	const FileMap &params;

};

}

}
