#pragma once

#include <unordered_set>
#include <unordered_map>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/GenericParam.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLStateValidityChecker.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class PlakuStateSampler : public AbstractionBasedSampler {

protected:
	struct Vertex {
		Vertex(unsigned int id) : heapIndex(std::numeric_limits<unsigned int>::max()), id(id),
			numSelections(0), heuristic(std::numeric_limits<double>::infinity()), weight(0), onOpen(false),
			currentParent(NULL) {}

		~Vertex() {
			for(auto p : parents) {
				delete p;
			}
		}

		void selected(double alpha) {
			numSelections++;
			weight = pow(alpha, numSelections) / (std::numeric_limits<double>::epsilon() + heuristic);
		}

		unsigned int getRandomRegionAlongPathToGoal(ompl::RNG &randomNumbers) const {
			unsigned int randomIndex = (unsigned int)(randomNumbers.uniform01() * regionPath.size());
			if(regionPath.size() == 0) {
				fprintf(stderr, "about to fail\n");
			}

			return regionPath[randomIndex];
		}

		unsigned int heapIndex;
		static bool pred(const Vertex *a, const Vertex *b) {
			return a->heuristic < b->heuristic;
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			r->heapIndex = i;
		}

		static bool HeapCompare(const Vertex *r1, const Vertex *r2) {
			return r1->weight < r2->weight;
		}

		unsigned int id, numSelections;
		double heuristic, weight;
		std::vector<unsigned int> regionPath;
		ompl::base::State *state;
		bool onOpen;

		struct Parent {
			Parent(unsigned int parent, double cost, const std::vector<unsigned int> &path) : parent(parent), cost(cost), path(path) {}
			static bool HeapCompare(const Parent *r1, const Parent *r2) {
				return r1->cost < r2->cost;
			}
			unsigned int parent;
			double cost;
			std::vector<unsigned int> path;
		};

		bool addParent(unsigned int parent, double cost, const std::vector<unsigned int> &path) {
			Parent *newParent = new Parent(parent, cost, path);
			newParent->path.emplace_back(id);

			if(currentParent == NULL) {
				currentParent = newParent;
				regionPath = currentParent->path;
				heuristic = currentParent->cost;
				return true;
			} else {
				if(cost < currentParent->cost) {
					parents.push_back(currentParent);
					std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
					currentParent = newParent;
					heuristic = currentParent->cost;
					regionPath = currentParent->path;
					return true;
				} else {
					parents.push_back(newParent);
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
				heuristic = std::numeric_limits<double>::infinity();
				regionPath.clear();
			} else {
				currentParent = parents.front();
				std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
				parents.pop_back();
				heuristic = currentParent->cost;
				regionPath = currentParent->path;
			}
		}

		std::vector<Parent *> parents;
		Parent *currentParent;
	};

public:
	PlakuStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start_, const ompl::base::GoalPtr &goal, const FileMap &params)
		: AbstractionBasedSampler(base, start_, goal, params), alpha(params.doubleVal("Alpha")), b(params.doubleVal("B")),
		  activeRegion(NULL), start(base->getStateSpace()->allocState()) {

		base->getStateSpace()->copyState(start, start_);
	}

	virtual ~PlakuStateSampler() {}

	virtual void initialize() {
		Timer timer("Abstraction Computation");

		abstraction->initialize();
		
		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.clear();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
		}

		startRegionId = abstraction->getStartIndex();
		goalRegionId = abstraction->getGoalIndex();
		dijkstra(&vertices[goalRegionId]);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[startRegionId].heuristic));

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].heuristic; }, "plaku.prm");

		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startID = abstraction->mapToAbstractRegion(incomingState);

		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].heuristic; }, vertices[startID].regionPath, "plakupath.prm");
#endif

		reached(start);
	}

	virtual bool sample(ompl::base::State *state) {
		if(activeRegion != NULL) {
			if(!activeRegion->onOpen) {
				regionHeap.push_back(activeRegion);
				std::push_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
				activeRegion->onOpen = true;
			}
		}

		if(randomNumbers.uniform01() < b) {
			assert(!regionHeap.empty());
			activeRegion = regionHeap.front();
			std::pop_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
			regionHeap.pop_back();
			activeRegion->selected(alpha);
			activeRegion->onOpen = false;
			unsigned int regionAlongPath = activeRegion->getRandomRegionAlongPathToGoal(randomNumbers);

			ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
			if(abstraction->supportsSampling()) {
				vertexState = abstraction->sampleAbstractState(regionAlongPath);
			}
			else {
				vertexState = abstraction->getState(regionAlongPath);
			}
			ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
			fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

			return true;
		} else {
			fullStateSampler->sampleUniform(state);
			return true;
		}
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("PlakuStateSampler::sampleNear", "not implemented");
		return false;
	}

	void reached(ompl::base::State *state) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);
		if(!vertices[newCellId].onOpen) {
			vertices[newCellId].selected(alpha);
			regionHeap.push_back(&vertices[newCellId]);
			std::push_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
			vertices[newCellId].onOpen = true;
		}
	}

protected:
	void dijkstra(Vertex *start) {
		Timer t("dijkstra");
		InPlaceBinaryHeap<Vertex, Vertex> open;
		std::unordered_set<unsigned int> closed;
		start->heuristic = 0;
		start->regionPath.emplace_back(start->id);
		open.push(start);

		closed.insert(start->id);

		while(!open.isEmpty()) {
			Vertex *current = open.pop();

			if(current->id != start->id) {
				unsigned int parentIndex = current->getBestParentIndex();
				if(!abstraction->isValidEdge(parentIndex, current->id)) {
					//this will update the value of the vertex if needed
					current->popBestParent();
					if(current->hasMoreParents()) {
						open.push(current);
					}
					continue;
				}
			}

			closed.insert(current->id);

			if(closed.size() == vertices.size()) break;

			std::vector<unsigned int> kids = getNeighboringCells(current->id);
			for(unsigned int kidIndex : kids) {
				if(closed.find(kidIndex) != closed.end()) continue;

				double newValue = current->heuristic + abstraction->abstractDistanceFunctionByIndex(current->id, kidIndex);

				Vertex *kid = &vertices[kidIndex];

				//this will update the value of the vertex if needed
				bool addedBetterParent = kid->addParent(current->id, newValue, current->regionPath);

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
	std::vector<Vertex*> regionHeap;
	ompl::RNG randomNumbers;

	ompl::base::State *start;
	unsigned int startRegionId, goalRegionId;
	Vertex *activeRegion;

	double alpha, b;
};

}

}
