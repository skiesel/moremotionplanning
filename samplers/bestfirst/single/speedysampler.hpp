#pragma once

#include "../../bestfirstsampler.hpp"

#include <ompl/base/objectives/StateCostIntegralObjective.h>

namespace ompl {

namespace base {

class SpeedySampler : public ompl::base::BestFirstSampler {

	struct ExtraData {
		ExtraData(double clearance) : clearance(clearance), heapIndex(std::numeric_limits<unsigned int>::max()) {}
		double e, clearance;
		unsigned int heapIndex;

		static bool pred(const Vertex *a, const Vertex *b) {
			return ((ExtraData *)a->extraData)->e < ((ExtraData *)b->extraData)->e;
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return ((ExtraData *)r->extraData)->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			((ExtraData *)r->extraData)->heapIndex = i;
		}
	};

	static inline ExtraData *extraData(Vertex *v) {
		return ((ExtraData *)v->extraData);
	}

	static inline const ExtraData *extraData(const Vertex *v) {
		return ((ExtraData *)v->extraData);
	}

	struct VertexEWrapper : public VertexWrapper {
		VertexEWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return extraData(vertex)->e;
		}
		void setVal(double c) {
			extraData(vertex)->e = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return extraData(this->vertex)->e < extraData(n2->vertex)->e;
		}
		bool operator<(const VertexWrapper *n2) const {
			return extraData(this->vertex)->e < extraData(n2->vertex)->e;
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

	virtual void generateVertices(const ompl::base::State *start, const ompl::base::State *goal, unsigned int howMany) {
		FBiasedStateSampler::generateVertices(start, goal, howMany);

		for(auto v : vertices) {
			v->extraData = new ExtraData(effortEstimator.stateCost(v->state).value());
		}
	}

	virtual void initialize() {
		auto abstractStart = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getStartState(0);
		auto abstractGoal = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState();

		bool connected = false;
		do {
			//Stolen from tools::SelfConfig::getDefaultNearestNeighbors
			if(si_->getStateSpace()->isMetricSpace()) {
				// if (specs.multithreaded)
				//  nn.reset(new NearestNeighborsGNAT<Vertex*>());
				//else
				nn.reset(new NearestNeighborsGNATNoThreadSafety<Vertex *>());
			} else {
				nn.reset(new NearestNeighborsSqrtApprox<Vertex *>());
			}

			nn->setDistanceFunction(boost::bind(&FBiasedStateSampler::abstractDistanceFunction, this, _1, _2));

			generateVertices(abstractStart, abstractGoal, prmSize);
			generateEdges(numEdges);

			std::vector<VertexWrapper *> wrappers;
			wrappers.reserve(vertices.size());
			std::vector<VertexEWrapper> eWrappers;
			eWrappers.reserve(vertices.size());
			for(auto v : vertices) {
				eWrappers.emplace_back(v);
				wrappers.emplace_back(&eWrappers.back());
			}

			dijkstra(wrappers[1], wrappers);

			connected = !(std::isinf(extraData(vertices[0])->e) || std::isinf(extraData(vertices[1])->e));

			if(!connected) {
				OMPL_INFORM("not connected! recomputing...");
				for(auto vert : vertices) {
					delete vert;
				}
				edges.clear();
				prmSize *= 1.5;
			}
		} while(!connected);

		auto neighbors = getNeighboringCells(0);
		for(auto n : neighbors) {
			open.push(vertices[n]);
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
		extraData(target)->e *= peekPenalty;
		open.siftFromItem(target);

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = target->state;

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

		Vertex v(0);
		auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
		v.state = ss.get();
		Vertex *reachedVertex = nn->nearest(&v);

		auto neighbors = getNeighboringCells(reachedVertex->id);
		for(auto n : neighbors) {
			if(!open.inHeap(vertices[n])) {
				open.push(vertices[n]);
			}
		}
	}

protected:
	double getEdgeEffortBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		double effort = (extraData(vertices[c1])->clearance + extraData(vertices[c2])->clearance) * edge->second.weight * 0.5;

		return effort;
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
				if(!isValidEdge(parentIndex, current->getId())) {
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

			std::vector<unsigned int> kids = getNeighboringCells(current->getId());
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

	InPlaceBinaryHeap<Vertex, ExtraData> open;
	AbstractEffortEstimator effortEstimator;
	bool useDNotE;
};

}

}