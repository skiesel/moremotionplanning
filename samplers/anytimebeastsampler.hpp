#pragma once

#include "beastsampler_dstar.hpp"
#include "../structs/gaussiandistribution.hpp"
#include <set>

namespace ompl {

namespace base {

class AnytimeBeastSampler : public ompl::base::BeastSampler_dstar {
public:
	AnytimeBeastSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
					BeastSampler_dstar(base, start, goal, gsr, params), optimizationObjective(optimizationObjective), goalPtr(goal) {

		probabilityThreshold = params.exists("ProbabilityThreshold") ? params.doubleVal("ProbabilityThreshold") : 0;
	}

	~AnytimeBeastSampler() {}

	virtual void initialize() {
		BeastSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		gCostDistributions.resize(abstractionSize);

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
			Timer t("Shortest Path Computation");
			computeShortestPath();
			dijkstra(startID, [](const Vertex* v){ return v->initG; }, [](Vertex* v, double val){ v->initG = val; });
			dijkstra(goalID,  [](const Vertex* v){ return v->initH; }, [](Vertex* v, double val){ v->initH = val; });
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				e.second->initialEffort = e.second->effort;
			}
		}

		vertices[startID].addUnsortedState(startState);


		addOutgoingEdgesToOpen(startID);
	}

	virtual bool sample(ompl::base::State *to, const base::PlannerTerminationCondition &ptc) {
		if(targetEdge != NULL) { //only will fail the first time through

			if(targetSuccess) {
				if(!addedGoalEdge && targetEdge->endID == goalID) {
					Edge *goalEdge = new Edge(goalID, goalID);
					goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
					goalEdge->effort = 1;
					focal.insert(goalEdge);
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

		assert(focal.size() > 0);
		bool shouldReset = false;
		unsigned int loops = 0;
		bool allOutsideProbabilityBound = false;
		for(auto iter = focal.begin(); ; ++iter) {
			auto atEnd = (iter == focal.end());
			if(atEnd || shouldReset) {
				if(atEnd && allOutsideProbabilityBound) {
					startOver();
					allOutsideProbabilityBound = false;
				}

				iter = focal.begin();
				shouldReset = false;

				if(loops++ > 10) {
					targetEdge = *iter;
					break;
				}
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
			} else if(vertices[targetEdge->startID].states.size() > 0) {
				auto expand = shouldExpand(targetEdge);
				bool withinProbabilityBound = expand.second >= probabilityThreshold;
				allOutsideProbabilityBound |= !withinProbabilityBound;
				if(withinProbabilityBound && expand.first) {
					break;
				}
			}

			if(ptc != false) {
				return false;
			}
		}

		targetSuccess = false;
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

	virtual void remove(ompl::base::State *state, double g) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int cellId = abstraction->mapToAbstractRegion(incomingState);
		vertices[cellId].removeUnsortedState(state);

		if(firstTargetSuccessState == state) {
			targetSuccess = false;
		}

		if(vertices[cellId].states.size() == 0) {
			for(auto e : reverseEdges[cellId]) {
				e.second->interior = false;
			}
		}

		gCostDistributions[cellId].removeDataPoint(g);
		if(cellId != startID) {
			errorDistribution.removeDataPoint(getError(vertices[cellId].initG, g));
		}
	}

	double getError(double startG, double foundG) const {
		return foundG / startG;
	}

	virtual void foundSolution(const ompl::base::Cost &incumbent) {
		incumbentCost = incumbent.value();
		startOver();
	}

	virtual void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

		incomingState = end;
		unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

		gCostDistributions[endCellId].addDataPoint(endG);
		
		if(endCellId != startID) {
			errorDistribution.addDataPoint(getError(vertices[endCellId].initG, endG));
		}

		vertices[endCellId].addUnsortedState(end);

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && endCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			if(firstTargetSuccessState == nullptr) {
				firstTargetSuccessState = end;
			}
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(endCellId);
		}
	}

	void hValueUpdate(ompl::base::State *state, double h) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int cellId = abstraction->mapToAbstractRegion(incomingState);
	}

protected:

	void startOver() {
		targetEdge = NULL;
		addedGoalEdge = false;
		focal.clear();

		for(unsigned int i = 0; i < abstraction->getAbstractionSize(); ++i) {
			vertices[i].clearStates();
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				getEdge(i, n)->interior = false;
				getEdge(n, i)->interior = false;;
			}
		}

		vertices[startID].addUnsortedState(startState);

		addOutgoingEdgesToOpen(startID);
	}

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
		auto iter = focal.find(e);
		bool onOpen = iter != focal.end();

		if(onOpen) {
			focal.erase(iter);
		}

		e->effort = effort;

		if(addToOpen || onOpen) {
			focal.insert(e);
		}
	}

	std::pair<bool, double> shouldExpand(const Edge* e) {
		if(std::isinf(incumbentCost)) {
			return std::make_pair(true, 1.);
		} else {
			double r = randomNumbers.uniform01();
// TODO:
// Should this be gCostDistributions[e->startID] + ?e->cost? + (errorDistribution * vertices[e->endID].initH);
			GaussianDistribution f = gCostDistributions[e->endID] + (errorDistribution * vertices[e->endID].initH);

			double val = f.getCDF(incumbentCost);
			return std::make_pair(r <= val, val);
		}
	}

	struct VertexWrapper {
		static std::function<double(const Vertex*)> getVal;
		static std::function<void(Vertex*, double)> setVal;

		static bool pred(const VertexWrapper *a, const VertexWrapper *b) {
			return getVal(a->v) < getVal(b->v);
		}
		static unsigned int getHeapIndex(const VertexWrapper *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(VertexWrapper *r, unsigned int i) {
			r->heapIndex = i;
		}
		unsigned int getId() const {
			return v->id;
		}
		unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
		Vertex *v = NULL;

		struct Parent {
			Parent(unsigned int parent, double cost) : parent(parent), cost(cost) {}
			static bool HeapCompare(const Parent *r1, const Parent *r2) {
				return r1->cost < r2->cost;
			}
			unsigned int parent;
			double cost;
		};

		bool addParent(unsigned int parent, double cost) {
			if(currentParent == NULL) {
				currentParent = new Parent(parent, cost);
				setVal(v, currentParent->cost);
				return true;
			} else {
				if(cost < currentParent->cost) {
					parents.push_back(currentParent);
					std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
					currentParent = new Parent(parent, cost);
					setVal(v, currentParent->cost);
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
				setVal(v, std::numeric_limits<double>::infinity());
			} else {
				currentParent = parents.front();
				std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
				parents.pop_back();
				setVal(v, currentParent->cost);
			}
		}

		std::vector<Parent *> parents;
		Parent *currentParent;
	};

	void dijkstra(unsigned int start, std::function<double(const Vertex*)> getVal, std::function<void(Vertex*, double)> setVal) {

		VertexWrapper::getVal = getVal;
		VertexWrapper::setVal = setVal;

		std::vector<VertexWrapper> wrappers(vertices.size());
		for(unsigned int i = 0; i < wrappers.size(); i++) {
			wrappers[i].v = &vertices[i];
		}

		InPlaceBinaryHeap<VertexWrapper, VertexWrapper> open;
		std::unordered_set<unsigned int> closed;
		setVal(wrappers[start].v, 0);
		open.push(&wrappers[start]);
		closed.insert(wrappers[start].getId());

		while(!open.isEmpty()) {
			VertexWrapper *current = open.pop();

			if(current->getId() != wrappers[start].getId()) {
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

				double newValue = current->v->initG + abstraction->abstractDistanceFunctionByIndex(current->getId(), kidIndex);
				VertexWrapper *kid = &wrappers[kidIndex];

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

	class EdgeComparator {
	public:
		bool operator()(const Edge *lhs, const Edge *rhs) const {
			return Edge::pred(lhs, rhs);
		}
	};

	double incumbentCost = std::numeric_limits<double>::infinity();
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
	std::vector<GaussianDistribution> gCostDistributions;
	GaussianDistribution errorDistribution;
	const ompl::base::GoalPtr &goalPtr; 

	ompl::base::State *firstTargetSuccessState = nullptr;

	std::set<Edge*, EdgeComparator> focal;
	double probabilityThreshold = 0;
};

std::function<double(const ompl::base::BeastSamplerBase::Vertex*)> AnytimeBeastSampler::VertexWrapper::getVal;
std::function<void(ompl::base::BeastSamplerBase::Vertex*, double)> AnytimeBeastSampler::VertexWrapper::setVal;

}

}
