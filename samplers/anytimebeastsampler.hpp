#pragma once

#include "beastsampler_dstar.hpp"
#include "../structs/gaussiandistribution.hpp"
#include <set>

namespace ompl {

namespace base {

class AnytimeBeastSampler : public ompl::base::BeastSampler_dstar {
protected:
	class DistributionTriple {
	public:
		void addGValue(double value) {
			g.addDataPoint(value);
		}

		void removeGValue(double value) {
			g.removeDataPoint(value);
		}
		
		void addHValue(double value) {
			h.addDataPoint(value);
		}

		void removeHValue(double value) {
			h.removeDataPoint(value);
		}

		double getProbabilityFLessThanEqual(double incumbent) {
			f.add(g, h);
			//really it's like (incumbent - epsilon) but it's not going to matter
			return f.getCDF(incumbent);
		}

	protected:
		GaussianDistribution g, h, f;
	};

public:
	AnytimeBeastSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
					BeastSampler_dstar(base, start, goal, gsr, params), optimizationObjective(optimizationObjective), goalPtr(goal) {}

	~AnytimeBeastSampler() {}

	virtual void initialize() {
		BeastSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		costDistributions.resize(abstractionSize);

		ompl::base::ScopedState<> startStateSS(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		ompl::base::ScopedState<> endStateSS(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				getEdge(i, n);
				getEdge(n, i);
			}
		}

		vertices[goalID].rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		U.push(&vertices[goalID]);

		{
			Timer t("D* lite");
			computeShortestPath();
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				e.second->initialEffort = e.second->effort;
			}
		}

		vertices[startID].addUnsortedState(startState);


		addOutgoingEdgesToOpen(startID);
	}

	virtual bool sample(ompl::base::State *to) {
		if(targetEdge != NULL) { //only will fail the first time through

			if(targetSuccess) {
				if(!addedGoalEdge && targetEdge->endID == goalID) {
					Edge *goalEdge = new Edge(goalID, goalID);
					goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
					goalEdge->effort = 1;
					iterableOpen.insert(goalEdge);
					addedGoalEdge = true;
				}

				if(targetEdge->interior) {
					updateSuccesfulInteriorEdgePropagation(targetEdge);
					updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
				} else {
					//edge has become interior
					targetEdge->interior = true;
					targetEdge->succesfulPropagation();
					updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
				}
			} else {
				targetEdge->failurePropagation();
				updateEdgeEffort(targetEdge, targetEdge->getEstimatedRequiredSamples() + vertices[targetEdge->endID].g);
			}

			updateVertex(targetEdge->startID);
			computeShortestPath();

			if(targetSuccess) {
				addOutgoingEdgesToOpen(targetEdge->endID);
			}
		}

		assert(iterableOpen.size() > 0);
		bool shouldReset = false;
		for(auto iter = iterableOpen.begin(); ; ++iter) {
			if(iter == iterableOpen.end() || shouldReset) {
				iter = iterableOpen.begin();
				shouldReset = false;
			}

			targetEdge = *iter;

			if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
																																		Abstraction::Edge::INVALID;
				targetEdge->updateEdgeStatusKnowledge(status);

				updateVertex(targetEdge->startID);
				computeShortestPath();

				//according to the docs, we shouldn't need to, but for the safety of the iterator, reset it :-(
				shouldReset = true;
			} else if(vertices[targetEdge->startID].states.size() > 0 && shouldExpand(targetEdge)) {
				break;
			}
		}

		targetSuccess = false;

		if(targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
			// si_->copyState(from, vertices[targetEdge->startID].sampleState());
			goalSampler->sampleGoal(to);
		} else {
			// si_->copyState(from, vertices[targetEdge->startID].sampleState());
			ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
			if(abstraction->supportsSampling()) {
				vertexState = abstraction->sampleAbstractState(targetEdge->endID);
			} else {
				vertexState = abstraction->getState(targetEdge->endID);
				ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
				fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
			}
		}
		return true;
	}

	void remove(ompl::base::State *state, double g) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int cellId = abstraction->mapToAbstractRegion(incomingState);
		vertices[cellId].removeUnsortedState(state);
		costDistributions[cellId].removeGValue(g);
	}

	void foundSolution(const ompl::base::Cost &incumbent) {
		incumbentCost = incumbent.value();
		targetEdge = NULL;
		addedGoalEdge = false;
		iterableOpen.clear();

		for(auto &v : vertices) {
			v.clearStates();
		}

		vertices[startID].addUnsortedState(startState);

		addOutgoingEdgesToOpen(startID);
	}

	void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

		incomingState = end;
		unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

		costDistributions[endCellId].addGValue(endG);

		vertices[endCellId].addUnsortedState(end);

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && endCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(endCellId);
		}
	}

	void hValueUpdate(ompl::base::State *state, double h) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int cellId = abstraction->mapToAbstractRegion(incomingState);

		costDistributions[cellId].addHValue(h);
	}

protected:

	virtual void addOutgoingEdgesToOpen(unsigned int source) {
		auto neighbors = abstraction->getNeighboringCells(source);
		for(auto n : neighbors) {
			Edge *e = getEdge(source, n);
			if(std::isinf(vertices[n].g)) {
				vertexHasInfiniteValue(n);
			}
			updateEdgeEffort(e, e->getEstimatedRequiredSamples() + vertices[n].g);
		}
	}

	virtual void updateEdgeEffort(Edge *e, double effort, bool addToOpen = true) {
		auto iter = iterableOpen.find(e);
		bool onOpen = iter != iterableOpen.end();

		if(onOpen) {
			iterableOpen.erase(iter);
		}

		e->effort = effort;

		if(addToOpen || onOpen) {
			iterableOpen.insert(e);
		}
	}

	bool shouldExpand(const Edge* e) {
		return std::isinf(incumbentCost) || randomNumbers.uniform01() <= costDistributions[e->endID].getProbabilityFLessThanEqual(incumbentCost);
	}

	class EdgeComparator {
	public:
		bool operator()(const Edge *lhs, const Edge *rhs) const {
			// This is so WILDLY unlikely, but the way we're abusing std::set
			// we need to make sure that we never give the impression of equality
			// otherwise we won't actually add the Edge, however, this will maintain
			// that we only add each Edge once which is also what we want
			if(lhs->effort == rhs->effort) {
				return lhs < rhs;
			}
			return lhs->effort < rhs->effort;
		}
	};

	double incumbentCost = std::numeric_limits<double>::infinity();
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
	std::vector<DistributionTriple> costDistributions;
	const ompl::base::GoalPtr &goalPtr; 

	std::set<Edge*, EdgeComparator> iterableOpen;
};

}

}
