#pragma once

#include "../../bestfirstsampler.hpp"

namespace ompl {

namespace base {

class AstarSampler : public ompl::base::BestFirstSampler {
	enum LABELS {
		G,
		H,
		F,
		SCORE,
		TOTAL,
	};

protected:
	struct Vertex : public FBiasedStateSampler::Vertex {
		Vertex(unsigned int id, unsigned int numVals = 0) : FBiasedStateSampler::Vertex(id, numVals),
			heapIndex(std::numeric_limits<unsigned int>::max()) {}

		static bool pred(const Vertex *a, const Vertex *b) {
			if(a->vals[F] == b->vals[F]) {
				return a->vals[G] > b->vals[G];
			}
			return a->vals[F] < b->vals[F];
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			r->heapIndex = i;
		}

		unsigned int heapIndex;
	};

public:

	AstarSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	             const FileMap &params) :
		BestFirstSampler(base, start, goal, params) {
		weight = params.exists("Weight") ? params.doubleVal("Weight") : 1.0;
	}

	virtual ~AstarSampler() {}

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

		unsigned int startIndex = abstraction->getStartIndex();
		dijkstra(wrappers[startIndex], wrappers);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[abstraction->getGoalIndex()].vals[G]));

		std::vector<VertexHWrapper> hWrappers;
		hWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			hWrappers.emplace_back(&vertices[i]);
			wrappers[i] = &hWrappers.back();
		}

		dijkstra(wrappers[abstraction->getGoalIndex()], wrappers);

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].vals[G] + weight * vertices[vertex].vals[H]; }, "astar.prm");
#endif

		vertices[startIndex].vals[F] = vertices[startIndex].vals[G] + weight * vertices[startIndex].vals[H];

		auto neighbors = abstraction->getNeighboringCells(abstraction->getStartIndex());
		for(auto n : neighbors) {
			vertices[n].vals[F] = vertices[n].vals[G] + weight * vertices[n].vals[H];
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
		target->vals[F] *= peekPenalty;
		open.siftFromItem(target);

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = abstraction->getState(target->id);

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("AstarSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *fromState, ompl::base::State *toState) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = fromState;
		unsigned int fromIndex = abstraction->mapToAbstractRegion(incomingState);

		incomingState = toState;
		unsigned int reachedIndex = abstraction->mapToAbstractRegion(incomingState);

		double newG = vertices[fromIndex].vals[G] + abstraction->abstractDistanceFunctionByIndex(fromIndex, reachedIndex);

		Vertex &reachedRef = vertices[reachedIndex];
		Vertex *reachedPtr = &vertices[reachedIndex];

		if(newG < reachedRef.vals[G]) {
			reachedRef.vals[G] = newG;
			reachedRef.vals[F] = newG + weight * reachedRef.vals[H];

			if(open.inHeap(reachedPtr)) {
				open.siftFromItem(reachedPtr);
			} else {
				open.push(reachedPtr);
			}
		} else {
			if(open.inHeap(reachedPtr)) {
				open.remove(reachedPtr);
			}
		}

		auto neighbors = getNeighboringCells(reachedIndex);
		for(auto n : neighbors) {
			double newChildG = newG + abstraction->abstractDistanceFunctionByIndex(reachedIndex, n);

			Vertex &childRef = vertices[n];
			Vertex *childPtr = &vertices[n];

			if(newChildG < childRef.vals[G]) {
				childRef.vals[G] = newChildG;
				childRef.vals[F] = childRef.vals[G] + weight * childRef.vals[H];

				if(open.inHeap(childPtr)) {
					open.siftFromItem(childPtr);
				} else {
					open.push(childPtr);
				}
			}
		}
	}

protected:
	std::vector<Vertex> vertices;
	InPlaceBinaryHeap<Vertex, Vertex> open;
	double weight;
	Vertex *target = NULL;
};

}

}