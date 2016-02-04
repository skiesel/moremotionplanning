#pragma once

#include "../../bestfirstsampler.hpp"

namespace ompl {

namespace base {

class DijkstraSampler : public ompl::base::BestFirstSampler {
	enum LABELS {
		G,
		F,
		TOTAL,
	};

protected:
	struct Vertex : public FBiasedStateSampler::Vertex {
		Vertex(unsigned int id, unsigned int numVals = 0) : FBiasedStateSampler::Vertex(id, numVals),
			heapIndex(std::numeric_limits<unsigned int>::max()) {}

		static bool pred(const Vertex *a, const Vertex *b) {
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
	DijkstraSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                const FileMap &params) :
		BestFirstSampler(base, start, goal, params) {
	}

	virtual ~DijkstraSampler() {}

	virtual void initialize() {
		abstraction->initialize();
		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i, TOTAL);
		}

		unsigned int startIndex = abstraction->getStartIndex();

		vertices[startIndex].vals[F] = vertices[startIndex].vals[G] = 0;
		auto neighbors = abstraction->getNeighboringCells(startIndex);
		for(auto n : neighbors) {
			vertices[n].vals[F] = vertices[n].vals[G] = vertices[startIndex].vals[G] + abstraction->abstractDistanceFunctionByIndex(startIndex, n);
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
		if(abstraction->supportsSampling()) {
			vertexState = abstraction->sampleAbstractState(target->id);
		}
		else {
			vertexState = abstraction->getState(target->id);
		}

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("DijkstraSampler::sampleNear", "not implemented");
		return false;
	}

	virtual void reached(ompl::base::State *fromState, ompl::base::State *toState) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = fromState;
		unsigned int fromIndex = abstraction->mapToAbstractRegion(incomingState);

		incomingState = toState;
		unsigned int reachedIndex = abstraction->mapToAbstractRegion(incomingState);

		double newG = vertices[fromIndex].vals[G] + abstraction->abstractDistanceFunctionByIndex(fromIndex, reachedIndex);

		Vertex &reachedVertexRef = vertices[reachedIndex];
		Vertex *reachedVertexPtr = &vertices[reachedIndex];

		if(newG < reachedVertexRef.vals[G]) {
			reachedVertexRef.vals[G] = reachedVertexRef.vals[F] = newG;

			if(open.inHeap(reachedVertexPtr)) {
				open.siftFromItem(reachedVertexPtr);
			} else {
				open.push(reachedVertexPtr);
			}
		} else {
			if(open.inHeap(reachedVertexPtr)) {
				open.remove(reachedVertexPtr);
			}
		}

		auto neighbors = abstraction->getNeighboringCells(reachedIndex);
		for(auto n : neighbors) {
			double newChildG = newG + abstraction->abstractDistanceFunctionByIndex(reachedIndex, n);
			Vertex &childVertexRef = vertices[n];
			Vertex *childVertexPtr = &vertices[n];


			if(newChildG < childVertexRef.vals[G]) {
				childVertexRef.vals[G] = childVertexRef.vals[F] = newChildG;

				if(open.inHeap(childVertexPtr)) {
					open.siftFromItem(childVertexPtr);
				} else {
					open.push(childVertexPtr);
				}
			}
		}
	}

protected:
	std::vector<Vertex> vertices;
	InPlaceBinaryHeap<Vertex, Vertex> open;
};

}

}