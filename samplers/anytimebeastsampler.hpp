#pragma once

/*

	After first solution is found
		Keep effort estimates, edges, states, etc (helpful knowledge)
		Empty open, add back start edges (restarts the search progress, kind of makes sense?)
		Only include edges within cost bound (keep the search focused)
			Are cost estimates admissible? (if not we can't prune using them)
			Can we prune?
*/

#include "beastsampler_dstar.hpp"

namespace ompl {

namespace base {

class AnytimeBeastSampler : public ompl::base::BeastSampler_dstar {
public:
	AnytimeBeastSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
					BeastSampler_dstar(base, start, goal, gsr, params), optimizationObjective(optimizationObjective) {}

	~AnytimeBeastSampler() {}

	virtual void initialize() {
		BeastSamplerBase::initialize();

		globalEdgeCostCorrection.addToAverage(1);

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
			const auto *startState = abstraction->getState(i);

			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				const auto *endState = abstraction->getState(n);
				ompl::base::Cost cost = optimizationObjective->motionCostHeuristic(startState, endState);

				auto *edgeA = getEdge(i, n);
				edgeA->initialEstimatedEdgeCost = cost.value();
				edgeA->updateEdgeCostHistory(cost.value());

				auto *edgeB = getEdge(n, i);
				edgeB->initialEstimatedEdgeCost = cost.value();
				edgeB->updateEdgeCostHistory(cost.value());
			}
		}

		vertices[goalID].rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		U.push(&vertices[goalID]);

		vertices[goalID].cost_rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		cost_U.push(&vertices[goalID]);

		{
			Timer t("D* lite");
			computeShortestPath();
			cost_computeShortestPath();
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				e.second->initialEffort = e.second->effort;
			}
		}

		vertices[startID].addState(startState);
		addOutgoingEdgesToOpen(startID);
	}

	void remove(ompl::base::State *state) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);
		vertices[newCellId].removeState(state);
	}

	void foundSolution(const ompl::base::Cost &incumbent) {
		incumbentCost = incumbent.value();
		targetEdge = NULL;
		addedGoalEdge = false;
		open.clear();
		addOutgoingEdgesToOpen(startID);
	}

	virtual bool sample(ompl::base::State *from, ompl::base::State *to) {
#ifdef STREAM_GRAPHICS
		// static unsigned int sampleCount = 0;
		// if(sampleCount++ % 100 == 0) {
			// fprintf(stderr, "open: %u\n", open.getFill());
			// writeVertexFile(sampleCount / 1000);
			// writeOpenEdgeFile(sampleCount / 1000);
			// writeUpdatedEdgeFile(sampleCount / 10);
		// 	writeEdgeFile(sampleCount / 100, 1);
		// }
#endif

		//This will fail when we're (res)starting or if the target edge start vertex was cleaned out due to SST pruning
		if(targetEdge != NULL && vertices[targetEdge->startID].states.size() > 1) {
			if(targetSuccess) {
				if(!addedGoalEdge && targetEdge->endID == goalID) {
					Edge *goalEdge = new Edge(goalID, goalID);
					goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
					goalEdge->effort = 1;
					open.push(goalEdge);
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
		while(true) {
			assert(!open.isEmpty());

			targetEdge = open.peek();

			if(vertices[targetEdge->startID].states.size() <= 0 || targetEdge->getEstimatedFCost() > incumbentCost) {
				open.pop();
			} else if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
																																		Abstraction::Edge::INVALID;
				targetEdge->updateEdgeStatusKnowledge(status);

				//yes this looks weird but we need it for right now to do some debugging
				updateEdgeEffort(targetEdge, targetEdge->effort);

				updateVertex(targetEdge->startID);
				computeShortestPath();
			} else {
				break;
			}
		}
		targetSuccess = false;

		if(targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
			goalSampler->sampleGoal(to);
		} else {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
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

	void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

		incomingState = end;
		unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

		vertices[endCellId].updateGCostHistory(endG);
		double newG = vertices[endCellId].getAverageGCost();
		for(auto &edge : edges[endCellId]) {
			edge.second->estimatedGCost = newG;
		}

		if(abstraction->edgeExists(startCellId, endCellId)) {
			auto *e = getEdge(startCellId, endCellId);
			double edgeCost = endG - startG;
			globalEdgeCostCorrection.addToAverage(edgeCost / e->initialEstimatedEdgeCost);
			e->updateEdgeCostHistory(edgeCost);
			cost_updateVertex(e->startID);
			cost_computeShortestPath();
		}

		vertices[endCellId].addState(end);

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && endCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(endCellId);
		}
	}

protected:

	void updateEdgeEffort(Edge *e, double effort, bool addToOpen = true) {
		e->effort = effort;
		if(addToOpen && e->getEstimatedFCost() < incumbentCost) {
			if(!open.inHeap(e)) {
				open.push(e);
			} else {
				open.siftFromItem(e);
			}
		}
	}

	Key cost_calculateKey(unsigned int id) {
		Vertex &s = vertices[id];
		Key key;
		key.first = std::min(s.cost_g, s.cost_rhs);
		key.second = std::min(s.cost_g, s.cost_rhs);
		return key;
	}

	void cost_updateVertex(unsigned int id) {
		Vertex &s = vertices[id];
		if(s.id != goalID) {
			double minValue = std::numeric_limits<double>::infinity();
			auto neighbors = abstraction->getNeighboringCells(id);
			for(auto n : neighbors) {
				Edge *e = getEdge(id, n);
				double value = vertices[n].cost_g + e->getEstimatedEdgeCost(globalEdgeCostCorrection.average);
				if(value < minValue) {
					minValue = value;
				}
			}
			s.cost_rhs = minValue;
		}

		if(cost_U.inHeap(&vertices[id])) {
			cost_U.remove(&vertices[id]);
		}

		if(s.cost_g != s.cost_rhs) {
			s.key = calculateKey(id);
			cost_U.push(&vertices[id]);
		}
	}

	void cost_computeShortestPath() {
		while(!cost_U.isEmpty()) {
			Vertex &u = vertices[cost_U.pop()->id];
			Key k_old = u.key;
			Key k_new = calculateKey(u.id);

			if(k_old < k_new) {
				u.key = k_new;
				cost_U.push(&vertices[u.id]);
			}
			else if(u.cost_g > u.cost_rhs) {
				u.cost_g = u.cost_rhs;
				for(auto e : reverseEdges[u.id]) {
					e.second->estimatedHCost = u.cost_g;
				}

				auto neighbors = getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			} else {
				u.cost_g = std::numeric_limits<double>::infinity();
				for(auto e : reverseEdges[u.id]) {
					e.second->estimatedHCost = u.cost_g;
				}

				updateVertex(u.id);
				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			}
		}
	}

	double incumbentCost = std::numeric_limits<double>::infinity();
	InPlaceBinaryHeap<Vertex, Vertex> cost_U;
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
	RunningAverage globalEdgeCostCorrection;
};

}

}
