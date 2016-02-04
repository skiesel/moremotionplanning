#pragma once

#include "../../bestfirstsampler.hpp"

#include <ompl/base/objectives/StateCostIntegralObjective.h>

namespace ompl {

namespace base {

class SpeedySampler : public ompl::base::BestFirstSampler {
	enum LABELS {
		CLEARANCE,
		E,
		TOTAL,
	};

protected:
	struct Vertex : public FBiasedStateSampler::Vertex {
		Vertex(unsigned int id, unsigned int numVals = 0) : FBiasedStateSampler::Vertex(id, numVals),
			heapIndex(std::numeric_limits<unsigned int>::max()) {}

		static bool pred(const Vertex *a, const Vertex *b) {
			return a->vals[E] < b->vals[E];
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			r->heapIndex = i;
		}

		unsigned int heapIndex;
	};


	struct VertexEWrapper : public VertexWrapper {
		VertexEWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return vertex->vals[E];
		}
		void setVal(double c) {
			vertex->vals[E] = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return vertex->vals[E] < n2->vertex->vals[E];
		}
		bool operator<(const VertexWrapper *n2) const {
			return vertex->vals[E] < n2->vertex->vals[E];
		}
	};

	class AbstractEffortEstimator : public ompl::base::StateCostIntegralObjective {
	public:
		AbstractEffortEstimator(const ompl::base::SpaceInformationPtr &si) :
			ompl::base::StateCostIntegralObjective(si, false) {}

		ompl::base::Cost stateCost(const ompl::base::State *s) const {
			return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
		}
	};

public:

	SpeedySampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	              const FileMap &params) :
		BestFirstSampler(base, start, goal, params),
		effortEstimator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()) {
		bool useD = params.exists("UseD") && (params.stringVal("UseD").compare("true") == 0);
		bool useE = params.exists("UseE") && (params.stringVal("UseE").compare("true") == 0);
		assert(useD != useE);
		useDNotE = useD;
	}

	virtual ~SpeedySampler() {}

	virtual void initialize() {
		abstraction->initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.clear();
		vertices.reserve(abstractionSize);
		if(useDNotE) {
			for(unsigned int i = 0; i < abstractionSize; ++i) {
				vertices.emplace_back(i, TOTAL);
			}
		} else {
			for(unsigned int i = 0; i < abstractionSize; ++i) {
				vertices.emplace_back(i, TOTAL);
				auto state = abstraction->getState(i);
				vertices.back().vals[CLEARANCE] = effortEstimator.stateCost(state).value();
			}
		}

		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(abstractionSize);
		std::vector<VertexEWrapper> eWrappers;
		eWrappers.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			eWrappers.emplace_back(&vertices[i]);
			wrappers.emplace_back(&eWrappers.back());
		}

		dijkstra(wrappers[abstraction->getGoalIndex()], wrappers);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[abstraction->getStartIndex()].vals[E]));

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].vals[E]; }, "speedy.prm");
#endif

		auto neighbors = abstraction->getNeighboringCells(0);
		for(auto n : neighbors) {
			open.push(&vertices[n]);
		}
	}

	virtual bool sample(ompl::base::State *state) {
		if(randomNumbers.uniform01() < randomStateProbability) {
			fullStateSampler->sampleUniform(state);
			return true;
		}

		if(open.isEmpty()) {
			static bool warned = false;
			if(!warned)
				OMPL_WARN("Open is empty!");
			warned = true;
			fullStateSampler->sampleUniform(state);
			return true;
		}

		Vertex *target = open.peek();
		target->vals[E] *= peekPenalty;
		open.siftFromItem(target);

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = abstraction->getState(target->id);

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("SpeedySampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *, ompl::base::State *toState) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = toState;
		unsigned int reachedIndex = abstraction->mapToAbstractRegion(incomingState);

		auto neighbors = abstraction->getNeighboringCells(reachedIndex);
		for(auto n : neighbors) {
			if(!open.inHeap(&vertices[n])) {
				open.push(&vertices[n]);
			}
		}
	}

protected:
	double getEdgeEffortBetweenCells(unsigned int c1, unsigned int c2) const {
		double dist = abstraction->abstractDistanceFunctionByIndex(c1, c2);
		return (vertices[c1].vals[CLEARANCE] + vertices[c2].vals[CLEARANCE]) * dist * 0.5;
	}

		virtual void dijkstra(VertexWrapper *start, const std::vector<VertexWrapper *> &wrappers) {
		Timer t("dijkstra");
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

				double newValue = current->getVal() + (useDNotE ? 1.0 : getEdgeEffortBetweenCells(current->getId(), kidIndex));
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

	std::vector<Vertex> vertices;
	InPlaceBinaryHeap<Vertex, Vertex> open;
	AbstractEffortEstimator effortEstimator;
	bool useDNotE;
};

}

}