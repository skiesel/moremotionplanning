#pragma once

#include "../structs/inplacebinaryheap.hpp"
#include "abstractionbasedsampler.hpp"

namespace ompl {

namespace base {

class BeastSamplerBase : public ompl::base::AbstractionBasedSampler {
	
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

		void removeState(const ompl::base::State *state) {
			bool found = false;
			for(unsigned int i = 0; i < states.size(); i++) {
				if(states[i].state == state) {
					states[i] = states.back();
					found = true;
					break;
				}
			}

			if(found) {
				states.pop_back();
				std::make_heap(states.begin(), states.end());
			}
		}

		void addUnsortedState(ompl::base::State *state) {
			states.emplace_back(state);
		}

		void removeUnsortedState(const ompl::base::State *state) {
			bool found = false;
			for(unsigned int i = 0; i < states.size(); i++) {
				if(states[i].state == state) {
					states[i] = states.back();
					found = true;
					break;
				}
			}

			if(found) {
				states.pop_back();
			}
		}

		void clearStates() {
			states.clear();
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
	

		unsigned int id;
		Key key;

		double initG = std::numeric_limits<double>::infinity();
		double initH = std::numeric_limits<double>::infinity();

		unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
		double g = std::numeric_limits<double>::infinity();
		double rhs = std::numeric_limits<double>::infinity();
	};

	struct Edge {
		Edge(unsigned int startID, unsigned int endID) : startID(startID), endID(endID) {}

		static double validEdgeDistributionAlpha, validEdgeDistributionBeta,
				invalidEdgeDistributionAlpha, invalidEdgeDistributionBeta;

		static bool pred(const Edge *lhs, const Edge *rhs) {
			if(lhs->effort != rhs->effort) {
				return lhs->effort < rhs->effort;
			}
			if(lhs->endID != rhs->endID) {
				return lhs->endID < rhs->endID;
			}
			if(lhs->startID != rhs->startID) {
				return lhs->startID < rhs->startID;
			}
			return false;
		}
		static unsigned int getHeapIndex(const Edge *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Edge *r, unsigned int i) {
			r->heapIndex = i;
		}

		double getEstimatedRequiredSamples() const {
			double probability = alpha / (alpha + beta);
			double estimate = 1. / probability;
			return estimate;
		}

		double getHypotheticalRequiredSamplesAfterPositivePropagation(unsigned int numberOfStates) const {
			double additive = (1. / (double)numberOfStates);
			double probability = (alpha + additive) / (alpha + additive + beta);
			double estimate = 1. / probability;
			return estimate;
		}

		void rewardHypotheticalSamplesAfterPositivePropagation(unsigned int numberOfStates) {
			double additive = (1. / ((double)numberOfStates - 1));
			alpha += additive;
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

		void print() const {
			fprintf(stderr, "%u -> %u (%g) +:%g -:%g\n", startID, endID, effort, alpha, beta);
		}

		unsigned int startID, endID, interiorToNextEdgeID;

		double alpha = validEdgeDistributionAlpha;
		double beta = validEdgeDistributionBeta;
		Abstraction::Edge::CollisionCheckingStatus status = Abstraction::Edge::UNKNOWN;
		unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
		double effort = std::numeric_limits<double>::infinity();
		double initialEffort = std::numeric_limits<double>::infinity();
		bool interior = false;
	};

public:
	BeastSamplerBase(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                         base::GoalSampleableRegion *gsr, const FileMap &params) : AbstractionBasedSampler(base, start, goal, params), goalSampler(gsr) {

		startState = base->allocState();
		si_->copyState(startState, start);

		goalState = base->allocState();
		si_->copyState(goalState, goal.get()->as<ompl::base::GoalState>()->getState());

		Edge::validEdgeDistributionAlpha = params.doubleVal("ValidEdgeDistributionAlpha");
		Edge::validEdgeDistributionBeta = params.doubleVal("ValidEdgeDistributionBeta");

		Edge::invalidEdgeDistributionAlpha = params.doubleVal("InvalidEdgeDistributionAlpha");
		Edge::invalidEdgeDistributionBeta = params.doubleVal("InvalidEdgeDistributionBeta");
	}

	virtual ~BeastSamplerBase() {}

	virtual void initialize() {
		abstraction->initialize();

		startID = abstraction->getStartIndex();
		goalID = abstraction->getGoalIndex();
	}

	void writeOpenEdgeFile(unsigned int outputNumber) {
		char buf[56];
		sprintf(buf, "effortOPEN%04u.edge", outputNumber);
		FILE *f = fopen(buf, "w");

		unsigned int size = abstraction->getAbstractionSize();

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();

		std::vector<Edge*> holder;
		while(!open.isEmpty()) {
			holder.push_back(open.pop());
			double val = holder.back()->effort;
			if(val < min) min = val;
			if(val > max) max = val;
		}

		for(auto *e : holder) {
			double val = e->effort;

			auto a = abstraction->getState(e->startID)->as<ompl::base::SE3StateSpace::StateType>();
			auto b = abstraction->getState(e->endID)->as<ompl::base::SE3StateSpace::StateType>();

			auto color = getColor(min, max, val);

			fprintf(f, "%g %g %g %g %g %g %g %g %g\n", a->getX(), a->getY(), a->getZ(), b->getX(), b->getY(), b->getZ(), color[0], color[1], color[2]);

			open.push(e);
		}

		fclose(f);
	}

	void writeEdgeFile(unsigned int outputNumber, unsigned int whichValue = 0) {
		char buf[56];
		sprintf(buf, "effortALL%04u.edge", outputNumber);
		FILE *f = fopen(buf, "w");

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();

		for(auto eset : edges) {
			for(auto e : eset.second) {
				double val = whichValue == 0 ? e.second->effort : e.second->getEstimatedRequiredSamples();

				if(std::isinf(val)) continue;
				if(val < min) min = val;
				if(val > max) max = val;
			}
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				double val = whichValue == 0 ? e.second->effort : e.second->getEstimatedRequiredSamples();

				if(std::isinf(val)) continue;

				auto a = abstraction->getState(e.second->startID)->as<ompl::base::SE3StateSpace::StateType>();
				auto b = abstraction->getState(e.second->endID)->as<ompl::base::SE3StateSpace::StateType>();

				auto color = getColor(min, max, val);

				fprintf(f, "%g %g %g %g %g %g %g %g %g\n", a->getX(), a->getY(), a->getZ(), b->getX(), b->getY(), b->getZ(), color[0], color[1], color[2]);
			}
		}

		fclose(f);
	}

	void writeUpdatedEdgeFile(unsigned int outputNumber) {
		char buf[56];
		sprintf(buf, "effortUPDATED%04u.edge", outputNumber);
		FILE *f = fopen(buf, "w");

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();

		for(auto eset : edges) {
			for(auto e : eset.second) {
				double val = e.second->initialEffort - e.second->effort;
				
				if(std::isinf(val) || val == 0) continue;

				if(val < min) min = val;
				if(val > max) max = val;
			}
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				double val = e.second->initialEffort - e.second->effort;

				if(std::isinf(val) || val == 0) continue;

				auto a = abstraction->getState(e.second->startID)->as<ompl::base::SE3StateSpace::StateType>();
				auto b = abstraction->getState(e.second->endID)->as<ompl::base::SE3StateSpace::StateType>();

				auto color = getColor(min, max, val);

				fprintf(f, "%g %g %g %g %g %g %g %g %g\n", a->getX(), a->getY(), a->getZ(), b->getX(), b->getY(), b->getZ(), color[0], color[1], color[2]);
			}
		}

		fclose(f);
	}

	void writeVertexFile(unsigned int outputNumber) const {
		char buf[56];
		sprintf(buf, "effort%04u.vert", outputNumber);
		FILE *f = fopen(buf, "w");

		unsigned int size = abstraction->getAbstractionSize();

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < size; ++i) {
			double val = vertices[i].g;

			if(std::isinf(val)) continue;

			if(val < min) min = val;
			if(val > max) max = val;
		}

		for(unsigned int i = 0; i < size; ++i) {
			double val = vertices[i].g;
			if(std::isinf(val)) continue;
			
			auto state = abstraction->getState(i)->as<ompl::base::SE3StateSpace::StateType>();

			auto color = getColor(min, max, val);

			fprintf(f, "%g %g %g %g %g %g\n", state->getX(), state->getY(), state->getZ(), color[0], color[1], color[2]);
		}

		fclose(f);
	}

	virtual bool sample(ompl::base::State *, ompl::base::State *) = 0;
	virtual bool sample(ompl::base::State *) = 0;
	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) = 0;
	virtual void reached(ompl::base::State *) = 0;

protected:

	virtual void vertexMayBeInconsistent(unsigned int) = 0;
	virtual void vertexHasInfiniteValue(unsigned int) = 0;

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

	Edge* getEdge(unsigned int a, unsigned int b) {
		Edge *e = edges[a][b];
		if(e == NULL) {
			e = new Edge(a, b);
			if(abstraction->getCollisionCheckStatusUnchecked(a, b) == Abstraction::Edge::INVALID) {
				e->updateEdgeStatusKnowledge(Abstraction::Edge::INVALID);
			}
			edges[a][b] = e;
			reverseEdges[b][a] = e;
		}
		return e;
	}

	virtual void updateEdgeEffort(Edge *e, double effort, bool addToOpen = true) {
		assert(effort >= 0);
		
		e->effort = effort;
		if(!open.inHeap(e) && addToOpen) {
			open.push(e);
		} else {
			open.siftFromItem(e);
		}
	}

	double getInteriorEdgeEffort(Edge *edge) {
		double mySamples = edge->getEstimatedRequiredSamples();
		double numberOfStates = vertices[edge->endID].states.size();

		double bestValue = std::numeric_limits<double>::infinity();
		auto neighbors = abstraction->getNeighboringCells(edge->endID);
		for(auto n : neighbors) {
			if(n == edge->startID) continue;

			Edge *e = getEdge(edge->endID, n);

			double value = mySamples + e->getHypotheticalRequiredSamplesAfterPositivePropagation(numberOfStates) +
				vertices[n].g;

			if(value < bestValue) {
				bestValue = value;
				edge->interiorToNextEdgeID = n;
			}
		}

		return bestValue;
	}

	void updateSuccesfulInteriorEdgePropagation(Edge *edge) {
		double numberOfStates = vertices[edge->endID].states.size();
		Edge *e = getEdge(edge->endID, edge->interiorToNextEdgeID);
		e->rewardHypotheticalSamplesAfterPositivePropagation(numberOfStates);
		vertexMayBeInconsistent(targetEdge->startID);
	}

	std::vector<Vertex> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge*>> edges;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge*>> reverseEdges;

	unsigned int startID, goalID;
	InPlaceBinaryHeap<Vertex, Vertex> U;
	InPlaceBinaryHeap<Edge, Edge> open;

	bool targetSuccess = false;
	Edge *targetEdge = NULL;
	ompl::base::State *startState = NULL;
	ompl::base::State *goalState = NULL;

	ompl::RNG randomNumbers;
	base::GoalSampleableRegion *goalSampler;
};

double BeastSamplerBase::Edge::validEdgeDistributionAlpha = 0;
double BeastSamplerBase::Edge::validEdgeDistributionBeta = 0;

double BeastSamplerBase::Edge::invalidEdgeDistributionAlpha = 0;
double BeastSamplerBase::Edge::invalidEdgeDistributionBeta = 0;

}

}
