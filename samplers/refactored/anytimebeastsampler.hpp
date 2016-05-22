#pragma once

#include <set>

#include "../abstractions/prmlite.hpp"

#include "abstractvertex.hpp"
#include "abstractedge.hpp"
#include "dijkstraable.hpp"
#include "dstarable.hpp"

namespace ompl {

namespace base {

namespace refactored {

class AnytimeBeastSampler : public ompl::base::UniformValidStateSampler {
	typedef AbstractVertex Vertex;
	typedef AbstractEdge Edge;
public:
	AnytimeBeastSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
					UniformValidStateSampler(base), fullStateSampler(base->allocStateSampler()), optimizationObjective(optimizationObjective), stateRadius(params.doubleVal("StateRadius")), goalSampler(gsr) {

		startState = base->allocState();
		base->copyState(startState, start);

		auto abstractStart = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getStartState(0);
		auto abstractGoal = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState();

		abstraction = new PRMLite(base, abstractStart, abstractGoal, params);

		Edge::validEdgeDistributionAlpha = params.doubleVal("ValidEdgeDistributionAlpha");
		Edge::validEdgeDistributionBeta = params.doubleVal("ValidEdgeDistributionBeta");

		Edge::invalidEdgeDistributionAlpha = params.doubleVal("InvalidEdgeDistributionAlpha");
		Edge::invalidEdgeDistributionBeta = params.doubleVal("InvalidEdgeDistributionBeta");
	}

	virtual ~AnytimeBeastSampler() {
		delete abstraction;
		delete dijkstra;
		delete dstar;
	}

	void initialize() {
		abstraction->initialize();

		startID = abstraction->getStartIndex();
		goalID = abstraction->getGoalIndex();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);
		gCostDistributions.resize(abstractionSize);

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				getEdge(i, n);
				getEdge(n, i);
			}
		}

		dstar = new DStarAble<Vertex, Edge>(vertices, goalID, abstraction,
			[&](unsigned int a, unsigned int b){ return getEdge(a, b); },
			[&](unsigned int id, double effortToGoal){ return DStarCallback(id, effortToGoal); });

		dijkstra = new DijkstraAble<Vertex>();

		{
			Timer t("Shortest Path Computation");
			
			dstar->computeShortestPath();
			
			dijkstra->dijkstra(startID, vertices, abstraction,
				[](const Vertex* v){ return v->initG; },
				[](Vertex* v, double val){ v->initG = val; });

			dijkstra->dijkstra(goalID, vertices, abstraction,
				[](const Vertex* v){ return v->initH; },
				[](Vertex* v, double val){ v->initH = val; });
		}

		vertices[startID].addState(startState);
		addOutgoingEdgesToOpen(startID);
	}

	bool sample(ompl::base::State *to, const base::PlannerTerminationCondition &ptc) {
		if(targetEdge != nullptr) {

			if(firstTargetSuccessState != nullptr) {
				targetSuccess();
			} else {
				targetFailure();
			}

			dstar->updateVertex(targetEdge->startID);
			dstar->computeShortestPath();

			if(firstTargetSuccessState != nullptr) {
				addOutgoingEdgesToOpen(targetEdge->endID);
			}
		}

		targetEdge = selectTargetEdge(ptc);
		if(targetEdge == nullptr) {
			return false;
		}

		firstTargetSuccessState = nullptr;

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

		if(firstTargetSuccessState == state) {
			firstTargetSuccessState = nullptr;
		}

		auto &vertex = vertices[cellId];

		ompl::base::State *copy = si_->allocState();
		si_->copyState(copy, state);
		for(auto *v : vertex.removedStates) {
			if(si_->equalStates(v, state)) {
				fprintf(stderr, "ALREADY REMOVED THIS STATE!!\n");
				assert(false);
			}
		}

		vertex.removedStates.emplace_back(copy);
		vertex.removeState(state);

		gCostDistributions[cellId].removeDataPoint(g);

		if(cellId != startID) {
			errorDistribution.removeDataPoint(getError(vertex.initG, g));
		}

		if(vertex.states.size() == 0) {
			for(auto e : reverseEdges[cellId]) {
				e.second->interior = false;
				dstar->updateVertex(e.second->startID);
			}
			dstar->computeShortestPath();
		}
	}

	double getError(double heuristicG, double realizedG) const {
		return realizedG / heuristicG;
	}

	void foundSolution(const ompl::base::Cost &incumbent) {
		incumbentCost = incumbent.value();

		targetEdge = NULL;
		addedGoalEdge = false;
		open.clear();

		for(unsigned int i = 0; i < abstraction->getAbstractionSize(); ++i) {
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				getEdge(i, n)->interior = false;
				getEdge(n, i)->interior = false;;
			}
		}
		addOutgoingEdgesToOpen(startID);
	}

	void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

		incomingState = end;
		unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

		auto &endVertex = vertices[endCellId];

		if(endVertex.states.find(end) != endVertex.states.end()) {
			si_->getStateSpace()->printState(end, std::cerr);
			si_->getStateSpace()->printState(*(endVertex.states.find(end)), std::cerr);
		}

		endVertex.addState(end);
		gCostDistributions[endCellId].addDataPoint(endG);
		if(endCellId != startID) {
			errorDistribution.removeDataPoint(getError(endVertex.initG, endG));
		}

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && endCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			if(firstTargetSuccessState == nullptr) {
				firstTargetSuccessState = end;
			}
		} else  {
			addOutgoingEdgesToOpen(endCellId);
		}
	}

	std::vector<unsigned int> getNeighboringCells(unsigned int index) const {
		return abstraction->getNeighboringCells(index);
	}

protected:
	Edge* getEdge(unsigned int a, unsigned int b) {
		Edge *e = edges[a][b];
		if(e == NULL) {
			e = new Edge(a, b);
			e->updateEdgeStatusKnowledge(abstraction->getCollisionCheckStatusUnchecked(a, b));
			edges[a][b] = e;
			reverseEdges[b][a] = e;
		}
		return e;
	}

	void DStarCallback(unsigned int id, double effortToGoal) {
		for(auto e : reverseEdges[id]) {
			if(e.second->interior) {
				updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
			}
			else {
				updateEdgeEffort(e.second, effortToGoal + e.second->getEstimatedRequiredSamples(), false);
			}
		}
	}

	void addOutgoingEdgesToOpen(unsigned int id) {
		auto neighbors = abstraction->getNeighboringCells(id);
		for(auto n : neighbors) {
			Edge *e = getEdge(id, n);
			if(std::isinf(dstar->getG(n))) {
				dstar->computeShortestPath();
			}
			updateEdgeEffort(e, e->getEstimatedRequiredSamples() + dstar->getG(n));
		}
	}

	void updateEdgeEffort(Edge *e, double effort, bool addToOpen = true) {
		assert(effort >= 0);
		
		auto iter = open.find(e);
		e->effort = effort;
		if(iter == open.end()) {
			if(addToOpen) {
				open.insert(e);
			}
		} else {
			open.erase(iter);
			open.insert(e);
		}
	}

	double getInteriorEdgeEffort(Edge *edge) {
		double mySamples = edge->getEstimatedRequiredSamples();
		double numberOfStates = vertices[edge->endID].states.size();

		double bestValue = std::numeric_limits<double>::infinity();
		auto neighbors = abstraction->getNeighboringCells(edge->endID);
		for(auto n : neighbors) {
			if(n == edge->startID) continue;

			Edge *e = getEdge(edge->endID, n);

			double value = mySamples + e->getHypotheticalEstimatedSamplesAfterPositivePropagation(numberOfStates) +
				dstar->getG(n);

			if(value < bestValue) {
				bestValue = value;
				edge->interiorToNextEdgeID = n;
			}
		}

		return bestValue;
	}

	void updateSuccesfulInteriorEdgePropagation(Edge *edge) {
		double numberOfStates = vertices[edge->endID].states.size();
		Edge *e = getEdge(edge->endID, edge->interiorToNextEdgeID);
		e->rewardHypotheticalSamplesAfterPositivePropagation(numberOfStates);
		dstar->updateVertex(targetEdge->startID);
	}

	void targetSuccess() {
		if(!addedGoalEdge && targetEdge->endID == goalID) {
			Edge *goalEdge = new Edge(goalID, goalID);
			goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
			goalEdge->effort = 1;
			open.insert(goalEdge);
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
	}

	void targetFailure() {
		targetEdge->failurePropagation();
		updateEdgeEffort(targetEdge, targetEdge->getEstimatedRequiredSamples() + dstar->getG(targetEdge->endID));
	}

	Edge* selectTargetEdge(const base::PlannerTerminationCondition &ptc) {
		bool shouldReset = false;
		for(auto iter = open.begin(); ; ++iter) {
			if(iter == open.end()) {
				iter = open.begin();
			}

			targetEdge = *iter;

			if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				if(abstraction->isValidEdge(targetEdge->startID, targetEdge->endID)) {
					targetEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
				} else {
					targetEdge->updateEdgeStatusKnowledge(Abstraction::Edge::INVALID);
				}

				dstar->updateVertex(targetEdge->startID);
				dstar->computeShortestPath();

			} else if(/*vertices[targetEdge->startID].states.size() > 0 &&*/ shouldExpand(targetEdge)) {
				return targetEdge;
			}

			if(ptc != false) {
				return nullptr;
			}
		}
	}

	bool shouldExpand(const Edge* e) {
		if(std::isinf(incumbentCost)) {
			return true;
		} else {
			double r = randomNumbers.uniform01();
			GaussianDistribution f = gCostDistributions[e->endID] + (errorDistribution * vertices[e->endID].initH);
			return r < f.getCDF(incumbentCost);
		}
	}


	Abstraction *abstraction = nullptr;
	unsigned int startID, goalID;
	StateSamplerPtr fullStateSampler;
	base::GoalSampleableRegion *goalSampler;

	ompl::base::State *startState = nullptr;

	ompl::RNG randomNumbers;

	Edge *targetEdge = nullptr;
	ompl::base::State *firstTargetSuccessState = nullptr;
	bool addedGoalEdge = false;

	double stateRadius;

	double incumbentCost = std::numeric_limits<double>::infinity();
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;

	std::vector<Vertex> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge*>> edges;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge*>> reverseEdges;

	std::set<Edge*, Edge::AbstractEdgeComparator> open;

	std::vector<GaussianDistribution> gCostDistributions;
	GaussianDistribution errorDistribution;

	DStarAble<Vertex, Edge> *dstar;
	DijkstraAble<Vertex> *dijkstra;
};

}

}

}