#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "../structs/filemap.hpp"
#include "../samplers/anytimebeastsampler.hpp"
#include "../samplers/anytimebeastsamplershim.hpp"

#include "modules/costpruningmodule.hpp"
#include "modules/sstpruningmodule.hpp"

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

		Planner::declareParam<bool>("intermediate_states", this, &AnytimeBeastPlanner::setIntermediateStates, &AnytimeBeastPlanner::getIntermediateStates);

		//Obviously this isn't really a parameter but I have no idea how else to get it into the output file through the benchmarker
		Planner::declareParam<double>("samplerinitializationtime", this, &AnytimeBeastPlanner::ignoreSetterDouble, &AnytimeBeastPlanner::getSamplerInitializationTime);
	}

	virtual ~AnytimeBeastPlanner() {
		freeMemory();
	}

	void ignoreSetterDouble(double) const {}
	void ignoreSetterUnsigedInt(unsigned int) const {}
	void ignoreSetterBool(bool) const {}	
	double getSamplerInitializationTime() const { return samplerInitializationTime; }

	bool getIntermediateStates() const { return addIntermediateStates_; }
	void setIntermediateStates(bool add) { addIntermediateStates_ = add; }

	virtual void setup() {
		ompl::control::RRT::setup();

	}

	virtual void clear() {
		RRT::clear();
		if(sstPruningModule != nullptr) {
			sstPruningModule->clear();
		}
	}

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		start = clock();

		checkValidity();
		base::Goal                   *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

		optimizationObjective = globalParameters.optimizationObjective;
		optimizationObjective->setCostThreshold(optimizationObjective->infiniteCost());



		if(sstPruningModule != nullptr) {}
		else if(params.stringVal("SSTStyle").compare("None") == 0) {
			sstPruningModule = new NoSSTPruningModule<MotionWithCost, Motion>();
		} else if(params.stringVal("SSTStyle").compare("SST") == 0) {
			sstPruningModule = new SSTPruningModule<MotionWithCost, Motion>(this, siC_, optimizationObjective,
				params.doubleVal("SelectionRadius"), params.doubleVal("PruningRadius"));
		} else if(params.stringVal("SSTStyle").compare("SSTStar") == 0) {
			sstPruningModule = new SSTStarPruningModule<MotionWithCost, Motion>(this, siC_, optimizationObjective,
				params.doubleVal("SelectionRadius"), params.doubleVal("PruningRadius"),
				params.doubleVal("Xi"), params.doubleVal("N0"),
				siC_->getStateSpace()->getDimension(),
				siC_->getControlSpace()->getDimension());
		} else {
			throw ompl::Exception("Unrecognized SSTStyle: %s", params.stringVal("SSTStyle"));
		}

		if(costPruningModule != nullptr) {}
		else if(params.stringVal("CostPruningStyle").compare("None") == 0) {
			costPruningModule = new NoCostPruningModule<MotionWithCost>(optimizationObjective);
		} else if(params.stringVal("CostPruningStyle").compare("G") == 0) {
			costPruningModule = new GCostPruningModule<MotionWithCost>(optimizationObjective);
		} else if(params.stringVal("CostPruningStyle").compare("F") == 0) {
			costPruningModule = new FCostPruningModule<MotionWithCost>(optimizationObjective, goal);
		} else {
			throw ompl::Exception("Unrecognized CostPruningStyle: %s", params.stringVal("CostPruningStyle"));
		}

		if(nn_->size() == 0) {
			while(const base::State *st = pis_.nextStart()) {
				MotionWithCost *motion = startState = new MotionWithCost(siC_);
				si_->copyState(motion->state, st);
				siC_->nullControl(motion->control);
				nn_->add(motion);

				sstPruningModule->addStartState(motion);
			}
		}

		if(nn_->size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		if(!newsampler) {
			auto start = clock();

			if(params.stringVal("Sampler").compare("BEAST") == 0) {
				newsampler = new ompl::base::AnytimeBeastSampler((ompl::base::SpaceInformation *)siC_, pdef_->getStartState(0), pdef_->getGoal(), goal_s, optimizationObjective, params);
				newsampler->initialize();
			} else {
				newsampler = new ompl::base::AnytimeBeastSamplerShim((ompl::base::SpaceInformation *)siC_, pdef_->getStartState(0), pdef_->getGoal(), goal_s, optimizationObjective, params, rng_);
			}

			samplerInitializationTime = (double)(clock() - start) / CLOCKS_PER_SEC;
		}

		if(!controlSampler_) {
			controlSampler_ = siC_->allocDirectedControlSampler();
		}

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

		MotionWithCost *rmotion = new MotionWithCost(siC_);
		base::State *rstate = rmotion->state;
		Control *rctrl = rmotion->control;

		while(ptc == false) {
			MotionWithCost *nmotion = NULL;

			if(!newsampler->sample(rstate, ptc)) {
				return ompl::base::PlannerStatus(false, false);
			}

			if(sstPruningModule->canSelectNode()) {
				nmotion = sstPruningModule->selectNode(rmotion, nn_);
			} else {
				nmotion = (MotionWithCost*)nn_->nearest(rmotion);
			}

			unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

			if(addIntermediateStates_) {
				std::vector<base::State *> pstates;
				cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

				int p = -1;
				if(cd >= siC_->getMinControlDuration()) {
					MotionWithCost *lastmotion = nmotion;

					bool solved = false;
					
					std::vector<MotionWithCost*> addedMotions;
					++p;
					for(; p < pstates.size(); ++p) {
						MotionWithCost *motion = new MotionWithCost();

						motion->state = pstates[p];

						motion->control = siC_->allocControl();
						siC_->copyControl(motion->control, rctrl);
						motion->steps = 1;
						lastmotion->numChildren++;
						motion->parent = lastmotion;
						motion->updateGValue(propagationStepSize);

						if(costPruningModule->shouldPrune(motion)) {
							break;
						}

						newsampler->reached(lastmotion->state, lastmotion->g.value(), pstates[p], motion->g.value());

						addedMotions.emplace_back(motion);
						nn_->add(motion);

						lastmotion = motion;

						double dist = 0.0;
						solved = goal->isSatisfied(motion->state, &dist);
						if(solved && optimizationObjective->isSatisfied(motion->g)) {
							globalParameters.solutionStream.addSolution(motion->g, start);
							newsampler->foundSolution(motion->g);
							optimizationObjective->setCostThreshold(motion->g);
							break;
						}
					}

					// post process the branch just added for SST* like pruning from leaf to root
					for (auto addedMotionsIterator = addedMotions.rbegin(); addedMotionsIterator != addedMotions.rend(); ++addedMotionsIterator) {
						MotionWithCost *motion = *addedMotionsIterator;
						MotionWithCost *pruned = sstPruningModule->shouldPrune(motion);
						if(pruned != nullptr) {
							nn_->remove(pruned);
							newsampler->remove(pruned->state, pruned->g.value());
							sstPruningModule->cleanupTree(pruned);
						}
					}
				}

				//free any states after we hit the goal
				//or all the states if we're not able to use
				//this propagation
				++p;
				for(; p < pstates.size(); ++p)
					si_->freeState(pstates[p]);

			} else {
				if(cd >= siC_->getMinControlDuration()) {
					/* create a motion */
					MotionWithCost *motion = new MotionWithCost(siC_);

					si_->copyState(motion->state, rmotion->state);
					siC_->copyControl(motion->control, rctrl);
					motion->steps = cd;
					nmotion->numChildren++;
					motion->parent = nmotion;
					motion->updateGValue(propagationStepSize);

					ompl::base::Cost h = optimizationObjective->costToGo(motion->state, goal);
					if(!costPruningModule->shouldPrune(motion)) {
						double dist = 0.0;
						bool solv = goal->isSatisfied(motion->state, &dist);
						if(solv && optimizationObjective->isSatisfied(motion->g)) {
							globalParameters.solutionStream.addSolution(motion->g, start);
							newsampler->foundSolution(motion->g);
							optimizationObjective->setCostThreshold(motion->g);
							break;
						}

						MotionWithCost *pruned = sstPruningModule->shouldPrune(motion);
						if(pruned != nullptr) {
							nn_->remove(pruned);
							newsampler->remove(pruned->state, pruned->g.value());
							sstPruningModule->cleanupTree(pruned);
						} else {
							newsampler->reached(nmotion->state, nmotion->g.value(),
												motion->state, motion->g.value());
							nn_->add(motion);
						}
					}
				}
			}
		}

		OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

		return ompl::base::PlannerStatus(false, false);
	}

protected:

	class MotionWithCost : public Motion {
	public:
		MotionWithCost() : Motion() {}

		MotionWithCost(const SpaceInformation *si) : Motion(si) {}

		void updateGValue(double propagationStepSize) {
			g = ompl::base::Cost(((MotionWithCost*)parent)->g.value() + propagationStepSize * steps);
		}

		ompl::base::Cost g = ompl::base::Cost(0);
		unsigned int numChildren = 0;
		bool inactive = false;
	};

	ompl::base::AnytimeBeastSampler *newsampler = NULL;

	
	base::OptimizationObjectivePtr optimizationObjective;
	double propagationStepSize, selectionRadius, pruningRadius, xi, n0, samplerInitializationTime = 0;
	clock_t start;

	const FileMap &params;
	CostPruningModule<MotionWithCost> *costPruningModule = NULL;
	SSTPruningModuleBase<MotionWithCost, Motion> *sstPruningModule = NULL;

	
	MotionWithCost *startState;
};

}

}


