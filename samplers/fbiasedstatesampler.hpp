#pragma once

#include "abstractionbasedsampler.hpp"
#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class FBiasedStateSampler : public AbstractionBasedSampler {
	enum LABELS {
		G,
		H,
		F,
		SCORE,
		TOTAL,
	};

protected:
	struct Vertex {
		Vertex() : id(std::numeric_limits<unsigned int>::max()) {}
		Vertex(unsigned int id, unsigned int numVals = 0) : id(id), vals(numVals, std::numeric_limits<double>::infinity()) {}
		unsigned int id;
		std::vector<double> vals;
	};

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

		virtual double getVal() const = 0;
		virtual void setVal(double) = 0;
		virtual bool sort(const VertexWrapper *) const = 0;

		static bool pred(const VertexWrapper *a, const VertexWrapper *b) {
			return a->sort(b);
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

	struct VertexGWrapper : public VertexWrapper {
		VertexGWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return vertex->vals[0];
		}
		void setVal(double c) {
			vertex->vals[G] = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return this->vertex->vals[G] < n2->vertex->vals[G];
		}
		bool operator<(const VertexWrapper *n2) const {
			return this->vertex->vals[G] < n2->vertex->vals[G];
		}
	};

	struct VertexHWrapper : public VertexWrapper {
		VertexHWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return vertex->vals[H];
		}
		void setVal(double c) {
			vertex->vals[H] = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return this->vertex->vals[H] < n2->vertex->vals[H];
		}
		bool operator<(const VertexWrapper *n2) const {
			return this->vertex->vals[H] < n2->vertex->vals[H];
		}
	};


public:
	FBiasedStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                    const FileMap &params, bool needOmega = true) : AbstractionBasedSampler(base, start, goal, params), omega(needOmega ? params.doubleVal("Omega") : 1) {}

	virtual ~FBiasedStateSampler() {}

	virtual void initialize() {
		abstraction->initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i, TOTAL);
		}

		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(vertices.size());
		std::vector<VertexGWrapper> gWrappers;
		gWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			gWrappers.emplace_back(&vertices[i]);
			wrappers.emplace_back(&gWrappers.back());
		}

		dijkstra(wrappers[abstraction->getStartIndex()], wrappers);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[abstraction->getGoalIndex()].vals[G]));

		std::vector<VertexHWrapper> hWrappers;
		hWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			hWrappers.emplace_back(&vertices[i]);
			wrappers[i] = &hWrappers.back();
		}

		dijkstra(wrappers[abstraction->getGoalIndex()], wrappers);

		generateRegionScores();

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			pdf.add(&vertices[i], vertices[i].vals[SCORE]);
		}

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].vals[SCORE]; }, "fbiased.prm");
#endif
	}

	virtual bool sample(ompl::base::State *state) {
		Vertex *randomVertex = pdf.sample();

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		
		if(abstraction->supportsSampling()) {
			vertexState = abstraction->sampleAbstractState(randomVertex->id);
		}
		else {
			vertexState = abstraction->getState(randomVertex->id);
		}

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("FBiasedStateSampler::sampleNear", "not implemented");
		return false;
	}

protected:
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

	void generateRegionScores() {
		double minF = std::numeric_limits<double>::infinity();
		std::vector<unsigned int> untouched;
		for(Vertex &n : vertices) {
			n.vals[F] = n.vals[G] + n.vals[H];
			if(n.vals[F] >= 0) {
				if(n.vals[F] < minF) {
					minF = n.vals[F];
				}
			} else {
				untouched.emplace_back(n.id);
			}
		}

		double numerator = pow(minF, omega);
		double minScore = std::numeric_limits<double>::infinity();
		double scoreSum = 0;
		for(Vertex &n : vertices) {
			if(n.vals[F] >= 0) {
				n.vals[SCORE] = numerator / pow(n.vals[F], omega);
				scoreSum += n.vals[SCORE];
				if(n.vals[SCORE] <= minScore) {
					minScore = n.vals[SCORE];
				}
			}
		}

		minScore /= 2;
		for(auto n : untouched) {
			vertices[n].vals[SCORE] = minScore;
		}
	}

	std::vector<Vertex> vertices;
	ProbabilityDensityFunction<Vertex> pdf;
	double omega;
};

}

}
