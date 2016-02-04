#pragma once

#include "abstractionbasedsampler.hpp"

namespace ompl {

namespace base {

class NewSampler : public ompl::base::AbstractionBasedSampler {
	
protected:
	struct Key {
		double first, second;
		bool operator<(const Key& k) const {
			if(first == k.first)
				return second < k.second;
			return first < k.first;
		}
	};

	struct Vertex {
		Vertex(unsigned int id) : id(id) {}

		static bool pred(const Vertex *a, const Vertex *b) {
			return a->key < b->key;
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			r->heapIndex = i;
		}

		struct StateWrapper {
			StateWrapper(ompl::base::State *state) : state(state) {}
			bool operator<(const StateWrapper &w) const { return selected < w.selected; }

			ompl::base::State *state;
			unsigned int selected = 0;
		};

		void addState(ompl::base::State *state) {
			states.emplace_back(state);
			std::push_heap(states.begin(), states.end());
		}

		ompl::base::State* sampleState() {
			auto state = states.front();
			std::pop_heap(states.begin(), states.end());
			states.pop_back();
			
			state.selected++;

			states.emplace_back(state);
			std::push_heap(states.begin(), states.end());
			
			return state.state;
		}

		std::vector<StateWrapper> states;
	

		unsigned int id, parentID;
		Key key;

		unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
		double g = std::numeric_limits<double>::infinity();
		double rhs = std::numeric_limits<double>::infinity();
	};

	struct Edge {
		Edge(unsigned int startID, unsigned int endID) : startID(startID), endID(endID) {}

		static double validEdgeDistributionAlpha, validEdgeDistributionBeta,
				invalidEdgeDistributionAlpha, invalidEdgeDistributionBeta;

		static bool pred(const Edge *a, const Edge *b) {
			return a->effort < b->effort;
		}
		static unsigned int getHeapIndex(const Edge *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Edge *r, unsigned int i) {
			r->heapIndex = i;
		}

		double getEstimatedRequiredSamples() const {
			double probability = alpha / (alpha + beta);
			double estimate = (1 - probability) / probability;
			return estimate > 1 ? estimate : 1;
		}

		double getHypotheticalRequiredSamplesAfterPositivePropagation(unsigned int numberOfStates) const {
			double additive = 1;
			double probability = (alpha + additive) / (alpha + additive + beta);
			double estimate = (1 - probability) / probability + numberOfStates - 1;
			return estimate > 1 ? estimate : 1;
		}

		void succesfulPropagation() {
			alpha++;
		}

		void failurePropagation() {
			beta++;
		}

		void updateEdgeStatusKnowledge(Abstraction::Edge::CollisionCheckingStatus status) {
			this->status = status;
			switch(status) {
				case Abstraction::Edge::VALID:
					alpha = validEdgeDistributionAlpha;
					beta = validEdgeDistributionBeta;
					break;
				case Abstraction::Edge::INVALID:
					alpha = invalidEdgeDistributionAlpha;
					beta = invalidEdgeDistributionBeta;
					break;

				case Abstraction::Edge::UNKNOWN:
					// alpha = unknownEdgeDistributionAlpha;
					// beta = unknownEdgeDistributionBeta;
					break;

			}
		}

		unsigned int startID, endID;

		double alpha = validEdgeDistributionAlpha;
		double beta = validEdgeDistributionBeta;
		Abstraction::Edge::CollisionCheckingStatus status = Abstraction::Edge::UNKNOWN;
		unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
		double effort = std::numeric_limits<double>::infinity();
		bool interior = false;
	};

public:
	NewSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                         const FileMap &params) : AbstractionBasedSampler(base, start, goal, params) {

		startState = base->allocState();
		si_->copyState(startState, start);

		Edge::validEdgeDistributionAlpha = params.doubleVal("ValidEdgeDistributionAlpha");
		Edge::validEdgeDistributionBeta = params.doubleVal("ValidEdgeDistributionBeta");

		Edge::invalidEdgeDistributionAlpha = params.doubleVal("InvalidEdgeDistributionAlpha");
		Edge::invalidEdgeDistributionBeta = params.doubleVal("InvalidEdgeDistributionBeta");
	}

	virtual ~NewSampler() {}

	virtual void initialize() {
		abstraction->initialize();

		startID = abstraction->getGoalIndex();
		goalID = abstraction->getStartIndex();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
		}

		vertices[startID].rhs = 0;
		vertices[startID].key = calculateKey(startID);
		U.push(&vertices[startID]);

		{
			Timer t("LPA*");
			computeShortestPath();
		}

		vertices[abstraction->getStartIndex()].addState(startState);
		addOutgoingEdgesToOpen(abstraction->getStartIndex());

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].g; }, "new.prm");
#endif
	}

	virtual bool sample(ompl::base::State *from, ompl::base::State *to) {
		if(targetEdge != NULL) { //only will fail the first time through

			if(targetSuccess) {
				fprintf(stderr, "success : %s\n", targetEdge->interior ? "interior" : "exterior");
				//edge has become interior
				targetEdge->interior = true;
				targetEdge->succesfulPropagation();
				targetEdge->effort = getInteriorEdgeEffort(targetEdge);
			} else {
				// fprintf(stderr, "failure\n");
				targetEdge->failurePropagation();

				targetEdge->effort = targetEdge->getEstimatedRequiredSamples() + vertices[targetEdge->endID].g;
			}

			updateVertex(targetEdge->endID);
			goalID = targetEdge->endID;
			computeShortestPath();

			open.push(targetEdge);
			if(targetSuccess) {
				addOutgoingEdgesToOpen(targetEdge->endID);
			}
		}

		bool getNextEdge = true;
		while(getNextEdge) {
			assert(!open.isEmpty());

			targetEdge = open.pop();

			if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
																																		Abstraction::Edge::INVALID;
				targetEdge->updateEdgeStatusKnowledge(status);
				
				updateVertex(targetEdge->endID);
				goalID = targetEdge->endID;
				computeShortestPath();

				open.push(targetEdge);
			} else {
				getNextEdge = false;
			}
		}

		// fprintf(stderr, "%u -> %u (%g)\n", targetEdge->startID, targetEdge->endID, targetEdge->effort);

		si_->copyState(from, vertices[targetEdge->startID].sampleState());

		targetSuccess = false;

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = abstraction->getState(targetEdge->endID);

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);

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

		// double multiplier = vertices[newCellId].states.size();
		// multiplier = (multiplier - 1) / multiplier;

		// for(auto e : edges[newCellId]) {
		// 	e.second->alpha *= multiplier;
		// 	e.second->beta *= multiplier;

		// 	updateVertex(e.second->endID);
		// }

		// computeShortestPath();

		if(newCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(newCellId);
		}
	}

protected:
	void addOutgoingEdgesToOpen(unsigned int source) {
		auto neighbors = abstraction->getNeighboringCells(source);
		for(auto n : neighbors) {
			Edge *e = getEdge(source, n);

			if(std::isinf(vertices[n].g)) {
				goalID = vertices[n].id;
				computeShortestPath();
			}

			if(!open.inHeap(e)) {
				open.push(e);
			} else {
				open.siftFromItem(e);
			}
		}
	}

	double heuristic(unsigned int a, unsigned int b) const {
		return 0;
	}

	Key calculateKey(unsigned int id) {
		Vertex &s = vertices[id];
		Key key;
		key.first = std::min(s.g, s.rhs) + heuristic(startID, id);
		key.second = std::min(s.g, s.rhs);
		return key;
	}

	void updateVertex(unsigned int id) {
		Vertex &s = vertices[id];

		if(s.id != startID) {
			unsigned int bestParent;
			double minValue = std::numeric_limits<double>::infinity();
			auto neighbors = abstraction->getNeighboringCells(id);
			for(auto n : neighbors) {
				Edge *e = getEdge(n, id);
				double value = vertices[n].g + e->getEstimatedRequiredSamples();
				if(value < minValue) {
					minValue = value;
					bestParent = n;
				}
			}
			s.rhs = minValue;
			s.parentID = bestParent;
		}

		if(U.inHeap(&vertices[id])) {
			U.remove(&vertices[id]);
		}

		if(s.g != s.rhs) {
			s.key = calculateKey(id);
			U.push(&vertices[id]);
		}
	}

	void computeShortestPath() {
		Key goalKey = calculateKey(goalID);
		while(!U.isEmpty() && (U.peek()->key < goalKey || vertices[goalID].g != vertices[goalID].rhs)) {
			Vertex &u = *U.pop();

			if(u.id != startID) {
				Edge *e = getEdge(u.parentID, u.id);
				if(e->status == Abstraction::Edge::UNKNOWN) {
					Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(e->startID, e->endID) ? Abstraction::Edge::VALID :
																														Abstraction::Edge::INVALID;
					e->updateEdgeStatusKnowledge(status);
					updateVertex(u.id);
					continue;
				}
			}

			if(u.g > u.rhs) {
				u.g = u.rhs;

				Edge *e = getEdge(u.id, u.parentID);
				if(e->interior) {
					e->effort = getInteriorEdgeEffort(e);
				} else {
					e->effort = e->getEstimatedRequiredSamples() + vertices[u.parentID].g;
				}

				if(open.inHeap(e)) {
					open.siftFromItem(e);
				}


				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			} else {
				u.g = std::numeric_limits<double>::infinity();
				updateVertex(u.id);
				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			}

			goalKey = calculateKey(goalID);
		}
	}

	Edge* getEdge(unsigned int a, unsigned int b) {
		Edge *e = edges[a][b];
		if(e == NULL) {
			e = new Edge(a, b);
			edges[a][b] = e;
		}
		return e;
	}

	double getInteriorEdgeEffort(Edge *edge) {
		double mySamples = edge->getEstimatedRequiredSamples();
		double numberOfStates = vertices[edge->endID].states.size();

		// fprintf(stderr, "%g\n", numberOfStates);

		double bestValue = std::numeric_limits<double>::infinity();
		auto neighbors = abstraction->getNeighboringCells(edge->endID);
		for(auto n : neighbors) {
			if(n == edge->startID) continue;

			Edge *e = getEdge(edge->endID, n);

			double value = mySamples + e->getHypotheticalRequiredSamplesAfterPositivePropagation(numberOfStates) +
				vertices[n].g;

			// fprintf(stderr, "\t%g\n", e->getEstimatedRequiredSamples() + vertices[n].g);

			if(value < bestValue) {
				bestValue = value;
			}
		}

		// fprintf(stderr, "\t%g\n", bestValue);

		return bestValue;
	}

	std::vector<Vertex> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge*>> edges;

	unsigned int startID, goalID;
	InPlaceBinaryHeap<Vertex, Vertex> U;
	InPlaceBinaryHeap<Edge, Edge> open;

	bool targetSuccess = false;
	Edge *targetEdge = NULL;
	ompl::base::State *startState = NULL;
};

double NewSampler::Edge::validEdgeDistributionAlpha = 0;
double NewSampler::Edge::validEdgeDistributionBeta = 0;

double NewSampler::Edge::invalidEdgeDistributionAlpha = 0;
double NewSampler::Edge::invalidEdgeDistributionBeta = 0;

}

}
