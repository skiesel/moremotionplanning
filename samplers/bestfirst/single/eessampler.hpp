#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "../../bestfirstsampler.hpp"
#include "../../../structs/rbtree.hpp"

namespace ompl {

namespace base {

class EESSampler : public ompl::base::BestFirstSampler {
protected:
	enum LABELS {
		G,
		H,
		F,
		E,
		FHAT,
		EHAT,
		CLEARANCE,
		TOTAL,
	};

	struct Vertex : public FBiasedStateSampler::Vertex {
		Vertex(unsigned int id, double clearance, unsigned int numVals = 0) : FBiasedStateSampler::Vertex(id, numVals) {
			vals[CLEARANCE] = clearance;
		}

		unsigned int fIndex = std::numeric_limits<unsigned int>::max();
		unsigned int eHatIndex = std::numeric_limits<unsigned int>::max();
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

	struct SortOnF {
		static unsigned int getHeapIndex(const Vertex *n) {
			return ((EESSampler::Vertex*)n)->fIndex;
		}
		static void setHeapIndex(Vertex *n, unsigned int index) {
			((EESSampler::Vertex*)n)->fIndex = index;
		}
		static bool pred(const Vertex *a, const Vertex *b) {
			if(a->vals[F] == b->vals[F])
				return a->vals[G] > b->vals[G];
			return a->vals[F] < b->vals[F];
		}
	};

	struct SortOnEHat {
		static unsigned int getHeapIndex(const Vertex *n) {
			return ((EESSampler::Vertex*)n)->eHatIndex;
		}
		static void setHeapIndex(Vertex *n, unsigned int index) {
			((EESSampler::Vertex*)n)->eHatIndex = index;
		}
		static bool pred(const Vertex *a, const Vertex *b) {
			if(a->vals[EHAT] == b->vals[EHAT])
				return a->vals[FHAT] < b->vals[FHAT];
			return a->vals[EHAT] < b->vals[EHAT];
		}
	};

	struct SortOnFHat {
		static int compare(const Vertex *a, const Vertex *b) {
			if(a->vals[FHAT] == b->vals[FHAT]) return a->vals[F] - b->vals[F];
			return a->vals[FHAT] - b->vals[FHAT];
		}
	};

	class AbstractEffortEstimator : public ompl::base::StateCostIntegralObjective {
	public:
		AbstractEffortEstimator(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateCostIntegralObjective(si, false) {}
		ompl::base::Cost stateCost(const ompl::base::State *s) const {
			return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
		}
	};

public:

	EESSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	           const FileMap &params) :
		BestFirstSampler(base, start, goal, params),
		effortEstimator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()) {
		heuristicCorrection = 1.0;
		weight = params.exists("Weight") ? params.doubleVal("Weight") : 1.0;
	}

	virtual ~EESSampler() {}

	virtual void initialize() {
		Timer timer("Abstraction Computation");

		abstraction->initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			double clearance = effortEstimator.stateCost(abstraction->getState(i)).value();
			vertices.emplace_back(i, clearance, TOTAL);
		}

		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(vertices.size());
		std::vector<VertexHWrapper> hWrappers;
		hWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			hWrappers.emplace_back(&vertices[i]);
			wrappers.emplace_back(&hWrappers.back());
		}

		useEdgeEffort = false;
		dijkstra(wrappers[1], wrappers);

		assert(!std::isinf(vertices[0].vals[H]));

		std::vector<VertexEWrapper> eWrappers;
		eWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			eWrappers.emplace_back(&vertices[i]);
			wrappers[i++] = &eWrappers.back();
		}

		useEdgeEffort = true;
		dijkstra(wrappers[1], wrappers);

		assert(!std::isinf(vertices[0].vals[E]));

		vertices[1].vals[E] = std::numeric_limits<double>::epsilon();

		vertices[0].vals[F] = 0;
		vertices[0].vals[F] = vertices[0].vals[H] * weight;
		vertices[0].vals[FHAT] = vertices[0].vals[H] * weight * heuristicCorrection;
		vertices[0].vals[EHAT] = vertices[0].vals[E];

		auto neighbors = abstraction->getNeighboringCells(0);
		for(auto n : neighbors) {
			vertices[n].vals[G] = vertices[0].vals[G] + abstraction->abstractDistanceFunctionByIndex(0, n);
			vertices[n].vals[F] = vertices[n].vals[G] + vertices[n].vals[H] * weight;
			vertices[n].vals[FHAT] = vertices[n].vals[G] + vertices[n].vals[H] * weight * heuristicCorrection;
			vertices[n].vals[EHAT] = vertices[n].vals[E];

			addToQueues(&vertices[n]);
		}
	}

	virtual bool sample(ompl::base::State *state) {
		if(randomNumbers.uniform01() <= randomStateProbability) {
			fullStateSampler->sampleUniform(state);
			return true;
		}

		Vertex *vertex = NULL;
		while(vertex == NULL) {
			vertex = selectVertex();
			//vertex = selectVertex_con();
			//vertex = selectVertex_opt();

			if(vertex == NULL) {
				break;
			} else if(vertex->vals[F] > incumbentCost) {
				vertex = NULL;
			}
		}

		if(vertex == NULL) {
			static bool warned = false;
			if(!warned)
				OMPL_WARN("Open is empty!");
			warned = true;
			fullStateSampler->sampleUniform(state);
			return true;
		}

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = abstraction->getState(vertex->id);

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("EESSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *fromState, ompl::base::State *toState) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = fromState;
		unsigned int fromIndex = abstraction->mapToAbstractRegion(incomingState);
		Vertex &fromVertex = vertices[fromIndex];

		incomingState = toState;
		unsigned int reachedIndex = abstraction->mapToAbstractRegion(incomingState);
		Vertex &reachedVertex = vertices[reachedIndex];

		double oldBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->vals[FHAT] * weight;

		double newG = fromVertex.vals[G];

		newG += abstraction->abstractDistanceFunctionByIndex(fromIndex, reachedIndex);

		if(newG < reachedVertex.vals[G]) {
			reachedVertex.vals[G] = newG;
			reachedVertex.vals[F] = reachedVertex.vals[G] + reachedVertex.vals[H] * weight;
			reachedVertex.vals[FHAT] = reachedVertex.vals[G] + reachedVertex.vals[H]  * weight * heuristicCorrection;
			reachedVertex.vals[EHAT] = reachedVertex.vals[E];


			addToQueues(&vertices[reachedIndex]);
		} else {
			removeFromQueues(&vertices[reachedIndex]);
		}

		auto neighbors = abstraction->getNeighboringCells(reachedIndex);
		for(auto n : neighbors) {
			double newChildG = newG + abstraction->abstractDistanceFunctionByIndex(reachedIndex, n);

			if(newChildG < vertices[n].vals[G] && newChildG < incumbentCost) {
				vertices[n].vals[G] = newChildG;
				vertices[n].vals[F] = vertices[n].vals[G] + vertices[n].vals[H] * weight;
				vertices[n].vals[FHAT] = vertices[n].vals[G] + vertices[n].vals[H] * weight * heuristicCorrection;
				vertices[n].vals[EHAT] = vertices[n].vals[E];

				addToQueues(&vertices[n]);
			}
		}

		double newBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->vals[FHAT] * weight;

		if(oldBestFHatVal != newBestFHatVal) {
			rebuildFocal();
		}
	}

protected:
	void removeFromQueues(Vertex *v) {
		if(cleanup.inHeap(v)) {
			cleanup.remove(v);
		}
		if(focal.inHeap(v)) {
			focal.remove(v);
		}
		open.remove(v);
	}

	Vertex *selectVertex() {
		Vertex *bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
		Vertex *bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
		Vertex *bestEhat = focal.isEmpty() ? NULL : focal.peek();

		if(bestF == NULL || bestFhat == NULL || bestEhat == NULL) {
			return NULL;
		}

		if(bestEhat->vals[FHAT] <= weight * bestF->vals[F]) {
			bestEhat->vals[E] *= peekPenalty;
			focal.siftFromItem(bestEhat);
			return bestEhat;
		} else if(bestFhat->vals[FHAT] <= weight * bestF->vals[F]) {
			return bestFhat;
		} else {
			return bestF;
		}
	}

	void rebuildFocal() {
		double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->vals[FHAT] * weight;
		focal.clear();
		auto includeThis = [bound](Vertex* n) {
			return n != NULL && n->vals[FHAT] <= bound;
		};
		std::vector<Vertex *> range = open.getContiguousRange(includeThis);
		focal.createFromVector(range);
	}

	void addToQueues(Vertex *n) {
		double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : open.peekLeftmost()->vals[FHAT] * weight;

		open.remove(n);
		open.insert(n);

		if(cleanup.inHeap(n)) {
			cleanup.siftFromItem(n);
		} else {
			cleanup.push(n);
		}
		if(n->vals[FHAT] <= bound) {
			if(focal.inHeap(n)) {
				focal.siftFromItem(n);
			} else {
				focal.push(n);
			}
		}
	}

	double getEdgeEffortBetweenCells(unsigned int c1, unsigned int c2) const {
		return (vertices[c1].vals[CLEARANCE] + vertices[c2].vals[CLEARANCE]) * abstraction->abstractDistanceFunctionByIndex(c1, c2) * 0.5;
	}

	bool useEdgeEffort = false;

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

				double newValue = current->getVal() + (useEdgeEffort ? getEdgeEffortBetweenCells(current->getId(), kidIndex) : abstraction->abstractDistanceFunctionByIndex(current->getId(), kidIndex));
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
	AbstractEffortEstimator effortEstimator;
	RBTree<Vertex, SortOnFHat> open;
	InPlaceBinaryHeap<Vertex, SortOnEHat> focal;
	InPlaceBinaryHeap<Vertex, SortOnF> cleanup;
	double incumbentCost, weight, heuristicCorrection;
};

}

}
