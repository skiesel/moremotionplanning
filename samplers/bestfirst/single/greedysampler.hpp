#pragma once

#include "../../bestfirstsampler.hpp"

namespace ompl {

namespace base {

class GreedySampler : public ompl::base::BestFirstSampler {
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
			return a->vals[H] < b->vals[H];
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

	GreedySampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	              const FileMap &params) : BestFirstSampler(base, start, goal, params) {}

	virtual ~GreedySampler() {}

	virtual void initialize() {
		abstraction->initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i, TOTAL);
		}

		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(abstractionSize);
		std::vector<VertexHWrapper> hWrappers;
		hWrappers.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			hWrappers.emplace_back(&vertices[i]);
			wrappers.emplace_back(&hWrappers.back());
		}

		dijkstra(wrappers[abstraction->getGoalIndex()], wrappers);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[abstraction->getStartIndex()].vals[H]));

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].vals[H]; }, "greedy.prm");
#endif

		auto neighbors = abstraction->getNeighboringCells(abstraction->getStartIndex());
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
		target->vals[H] *= peekPenalty;
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
	std::vector<Vertex> vertices;
	InPlaceBinaryHeap<Vertex, Vertex> open;
};

}

}