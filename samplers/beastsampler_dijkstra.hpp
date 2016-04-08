#pragma once

#include "beastsamplerbase.hpp"

namespace ompl {

namespace base {

class BeastSampler_dijkstra : public ompl::base::BeastSamplerBase {
	struct VertexWrapper {
		VertexWrapper(Vertex *vertex) : vertex(vertex), currentParent(NULL) {}

		virtual ~VertexWrapper() {
			for(auto p : parents) {
				delete p;
			}
		}

		inline unsigned int getId() const {
			return vertex->id;
		}

		virtual double getVal() const {
			return vertex->g;
		}
		virtual void setVal(double val) {
			vertex->g = val;
		}
		static bool pred(const VertexWrapper *a, const VertexWrapper *b) {
			return a->vertex->g < b->vertex->g;
		}
		static unsigned int getHeapIndex(const VertexWrapper *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(VertexWrapper *r, unsigned int i) {
			r->heapIndex = i;
		}

		Vertex *vertex;
		unsigned int heapIndex;

		struct Parent {
			Parent(unsigned int parent, double cost) : parent(parent), cost(cost) {}
			static bool HeapCompare(const Parent *r1, const Parent *r2) {
				return r1->cost < r2->cost;
			}
			unsigned int parent;
			double cost;
		};

		virtual bool addParent(unsigned int parent, double cost) {
			if(currentParent == NULL) {
				currentParent = new Parent(parent, cost);
				setVal(currentParent->cost);
				return true;
			} else {
				if(cost < currentParent->cost) {
					parents.push_back(currentParent);
					std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
					currentParent = new Parent(parent, cost);
					setVal(currentParent->cost);
					return true;
				} else {
					parents.push_back(new Parent(parent, cost));
					std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
					return false;
				}
			}
		}

		bool hasMoreParents() const {
			return !parents.empty() || currentParent != NULL;
		}

		unsigned int getBestParentIndex() const {
			assert(currentParent != NULL);
			return currentParent->parent;
		}

		void popBestParent() {
			assert(currentParent != NULL);

			delete currentParent;

			if(parents.size() == 0) {
				currentParent = NULL;
				setVal(std::numeric_limits<double>::infinity());
			} else {
				currentParent = parents.front();
				std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
				parents.pop_back();
				setVal(currentParent->cost);
			}
		}

		std::vector<Parent *> parents;
		Parent *currentParent;
	};
public:
	BeastSampler_dijkstra(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const FileMap &params) : BeastSamplerBase(base, start, goal, gsr, params) {}

	~BeastSampler_dijkstra() {}

	virtual void initialize() {
		BeastSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
		}

		{
			Timer t("dijkstra");
			dijkstra(goalID);
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

			dijkstra(goalID);

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

				dijkstra(goalID);
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
		dijkstra(goalID);
	}

	void vertexHasInfiniteValue(unsigned int id) {
		dijkstra(goalID);
	}

	void dijkstra(unsigned int startID) {
		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			wrappers.emplace_back(new VertexWrapper(&vertices[i]));
		}

		dijkstra(wrappers[startID], wrappers);
	}

	void dijkstra(VertexWrapper *start, const std::vector<VertexWrapper *> &wrappers) {
		InPlaceBinaryHeap<VertexWrapper, VertexWrapper> open;
		std::unordered_set<unsigned int> closed;
		start->setVal(0);
		open.push(start);
		closed.insert(start->getId());

		while(!open.isEmpty()) {
			VertexWrapper *current = open.pop();

			if(current->getId() != start->getId()) {
				unsigned int parentIndex = current->getBestParentIndex();
				if(!abstraction->isValidEdge(parentIndex, current->getId())) {
					//this will update the value of the vertex if needed
					current->popBestParent();
					if(current->hasMoreParents()) {
						open.push(current);
					}
					continue;
				}
			}

			closed.insert(current->getId());

			if(closed.size() == wrappers.size()) break;

			std::vector<unsigned int> kids = abstraction->getNeighboringCells(current->getId());
			for(unsigned int kidIndex : kids) {
				if(closed.find(kidIndex) != closed.end()) continue;

				double newValue = current->getVal() + abstraction->abstractDistanceFunctionByIndex(current->getId(), kidIndex);
				VertexWrapper *kid = wrappers[kidIndex];

				//this will update the value of the vertex if needed
				bool addedBetterParent = kid->addParent(current->getId(), newValue);

				if(open.inHeap(kid)) {
					if(addedBetterParent) {
						open.siftFromItem(kid);
					}
				} else {
					open.push(kid);
				}
			}
		}
	}
};

}

}
