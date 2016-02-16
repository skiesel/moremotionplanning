#pragma once

#include "newsamplerbase.hpp"

namespace ompl {

namespace base {

class NewSampler_dstar : public ompl::base::NewSamplerBase {
public:
	NewSampler_dstar(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const FileMap &params) : NewSamplerBase(base, start, goal, gsr, params) {}

	~NewSampler_dstar() {}

	virtual void initialize() {
		NewSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
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

		vertices[startID].addState(startState);
		addOutgoingEdgesToOpen(startID);
	}

	virtual bool sample(ompl::base::State *from, ompl::base::State *to) {
		// static unsigned int sampleCount = 0;
		// if(sampleCount++ % 100 == 0) {
		// 	// fprintf(stderr, "open: %u\n", open.getFill());
		// 	writeVertexFile(sampleCount / 100);
		// 	writeOpenEdgeFile(sampleCount / 100);
		// 	writeUpdatedEdgeFile(sampleCount / 100);
		// 	writeEdgeFile(sampleCount / 100);
		// }

		// for(unsigned int i = 1; i < open.fill; i++) {
		// 	auto l = open.left(i);
		// 	auto r = open.right(i);
		// 	if(l < open.fill && open.heap[i]->effort > open.heap[l]->effort) {
		// 		fprintf(stderr, "%g > %g\n", open.heap[i]->effort, open.heap[l]->effort);
		// 		open.siftFromItem(open.heap[l]);
		// 		i = 1;
		// 	}
		// 	if(r < open.fill && open.heap[i]->effort > open.heap[r]->effort) {
		// 		fprintf(stderr, "%g > %g\n", open.heap[i]->effort, open.heap[r]->effort);
		// 		open.siftFromItem(open.heap[r]);
		// 		i = 1;
		// 	}
		// }

		if(targetEdge != NULL) { //only will fail the first time through

			if(targetSuccess) {
				static bool addedGoalEdge = false;
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

		bool getNextEdge = true;
		while(getNextEdge) {
			assert(!open.isEmpty());

			targetEdge = open.peek();

			if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
																																		Abstraction::Edge::INVALID;
				targetEdge->updateEdgeStatusKnowledge(status);
				
				//yes this looks weird but we need it for right now to do some debugging
				updateEdgeEffort(targetEdge, targetEdge->effort);

				updateVertex(targetEdge->startID);
				computeShortestPath();
			} else {
				getNextEdge = false;
			}
		}

		targetSuccess = false;

		if(targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
			goalSampler->sampleGoal(to);
		} else {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
			ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
			vertexState = abstraction->getState(targetEdge->endID);

			ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
			fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
		}
		return true;
	}

	virtual bool sample(ompl::base::State *) {
		throw ompl::Exception("NewSampler::sample", "not implemented");
		return false;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("NewSampler::sampleNear", "not implemented");
		return false;
	}

	void reached(ompl::base::State *state) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);

		vertices[newCellId].addState(state);

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && newCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(newCellId);
		}
	}


protected:
	void vertexMayBeInconsistent(unsigned int id) {
		updateVertex(id);
	}

	void vertexHasInfiniteValue(unsigned int id) {
		computeShortestPath();
	}


	Key calculateKey(unsigned int id) {
		Vertex &s = vertices[id];
		Key key;
		key.first = std::min(s.g, s.rhs);
		key.second = std::min(s.g, s.rhs);
		return key;
	}

	void updateVertex(unsigned int id) {
		Vertex &s = vertices[id];
		if(s.id != goalID) {
			double minValue = std::numeric_limits<double>::infinity();
			auto neighbors = abstraction->getNeighboringCells(id);
			for(auto n : neighbors) {
				Edge *e = getEdge(id, n);
				double value = vertices[n].g + e->getEstimatedRequiredSamples();
				if(value < minValue) {
					minValue = value;
				}
			}
			s.rhs = minValue;
		}

		if(U.inHeap(&vertices[id])) {
			U.remove(&vertices[id]);
		}

		if(s.g != s.rhs) {
			s.key = calculateKey(id);
			U.push(&vertices[id]);
		}
	}

	void computeShortestPath() {
		while(!U.isEmpty()) {
			Vertex &u = vertices[U.pop()->id];
			Key k_old = u.key;
			Key k_new = calculateKey(u.id);

			if(k_old < k_new) {
				u.key = k_new;
				U.push(&vertices[u.id]);
			}
			else if(u.g > u.rhs) {
				u.g = u.rhs;
				for(auto e : reverseEdges[u.id]) {
					updateEdgeEffort(e.second, u.g + e.second->getEstimatedRequiredSamples(), false);
				}

				auto neighbors = getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			} else {
				u.g = std::numeric_limits<double>::infinity();

				for(auto e : reverseEdges[u.id]) {
					updateEdgeEffort(e.second, u.g, false);
				}

				updateVertex(u.id);
				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			}
		}
	}
};

}

}
