#pragma once

#include <unordered_map>
#include <unordered_set>

#include "../../domains/geometry/detail/FCLContinuousMotionValidator.hpp"

class Abstraction {
public:
	struct Vertex {
		Vertex(unsigned int id) : id(id), state(NULL), populatedNeighors(false) {}
		unsigned int id;
		ompl::base::State *state;
		std::vector<unsigned int> neighbors;
		bool populatedNeighors;
	};

	struct Edge {
		enum CollisionCheckingStatus {
			UNKNOWN = 0,
			INVALID = 1,
			VALID = 2,
		};

		Edge() : endpoint(std::numeric_limits<unsigned int>::max()), edgeStatus(INVALID) {}

		Edge(unsigned int endpoint) : endpoint(endpoint), edgeStatus(UNKNOWN) {}

		std::size_t operator()(const Edge &e) const { return e.endpoint; }

		bool operator==(const Edge &e) const { return e.endpoint == endpoint; }

		unsigned int endpoint;
		CollisionCheckingStatus edgeStatus;
	};

	Abstraction(const ompl::base::State *start, const ompl::base::State *goal) :
		motionValidator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->getMotionValidator()),
		start(start), goal(goal) {
	}

	virtual ~Abstraction() {
		for(auto vertex : vertices) {
			delete vertex;
		}
	}

	virtual void initialize(bool forceConnectedness = true) = 0;
	virtual unsigned int getStartIndex() const = 0;
	virtual unsigned int getGoalIndex() const = 0;
	virtual bool supportsSampling() const = 0;
	virtual ompl::base::State* sampleAbstractState(unsigned int index) = 0;
	virtual unsigned int mapToAbstractRegion(const ompl::base::ScopedState<> &s) const = 0;
	virtual void grow() = 0;

	unsigned int getAbstractionSize() const {
		return vertices.size();
	}

	const ompl::base::State* getState(unsigned int index) const {
		return vertices[index]->state;
	}

	bool edgeExists(unsigned int a, unsigned int b) {
		auto vertexAndEdges = edges.find(a);
		if(vertexAndEdges == edges.end()) {
			return false;
		}

		auto &edges = vertexAndEdges->second;

		auto edge = edges.find(b);
		if(edge == edges.end()) {
			return false;
		}
		return true;
	}

	bool isValidEdge(unsigned int a, unsigned int b) {
		auto vertexAndEdges = edges.find(a);
		if(vertexAndEdges == edges.end()) {
			return false;
		}

		auto &edges = vertexAndEdges->second;

		auto edge = edges.find(b);
		if(edge == edges.end()) {
			return false;
		}

		if(edge->second.edgeStatus == Edge::UNKNOWN) {
			// if(motionValidator->checkMotion(vertices[a]->state, vertices[b]->state)) {
			if(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->checkMotion(vertices[a]->state, vertices[b]->state)) {
				edge->second.edgeStatus = Edge::VALID;
				setEdgeStatusUnchecked(b, a, Edge::VALID);
			} else {
				edge->second.edgeStatus = Edge::INVALID;
				setEdgeStatusUnchecked(b, a, Edge::INVALID);
			}
		}

		return edge->second.edgeStatus == Edge::VALID;
	}

	bool isValidEdgeUnchecked(unsigned int a, unsigned int b) {
		auto &edge = edges[a][b];

		if(edge.edgeStatus == Edge::UNKNOWN) {
			// if(motionValidator->checkMotion(vertices[a]->state, vertices[b]->state)) {
			if(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->checkMotion(vertices[a]->state, vertices[b]->state)) {
				edge.edgeStatus = Edge::VALID;
				setEdgeStatusUnchecked(b, a, Edge::VALID);
			} else {
				edge.edgeStatus = Edge::INVALID;
				setEdgeStatusUnchecked(b, a, Edge::INVALID);
			}
		}

		return edge.edgeStatus == Edge::VALID;
	}

	Edge::CollisionCheckingStatus getCollisionCheckStatusUnchecked(unsigned int a, unsigned int b) {
		return edges[a][b].edgeStatus;
	}

	const std::vector<unsigned int>& getNeighboringCells(unsigned int index) {
		if(vertices[index]->populatedNeighors) {
			return vertices[index]->neighbors;
		}
		vertices[index]->populatedNeighors = true;

		auto vertexAndEdges = edges.find(index);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		vertices[index]->neighbors.reserve(edges.size());
		for(const auto &edge : edges) {
			vertices[index]->neighbors.emplace_back(edge.second.endpoint);
		}

		return vertices[index]->neighbors;
	}

	virtual double abstractDistanceFunction(const Vertex *a, const Vertex *b) const {
		return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(a->state, b->state);
	}

	virtual double abstractDistanceFunctionByIndex(unsigned int a, unsigned int b) const {
		return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(vertices[a]->state, vertices[b]->state);
	}

	bool checkConnectivity() {
		Timer("connectivity check");
		unsigned int startIndex = getStartIndex();
		unsigned int goalIndex = getGoalIndex();

		unsigned int index = 0;
		std::vector<unsigned int> open;
		std::unordered_set<unsigned int> closed;

		open.emplace_back(startIndex);
		while(index < open.size()) {
			unsigned int current = open[index];
			if(current == goalIndex) {
				return true;
			}
			auto neighbors = getNeighboringCells(current);
			for(auto n : neighbors) {
				if(closed.find(n) != closed.end()) continue;
				if(!isValidEdgeUnchecked(current, n)) continue;
				closed.insert(n);
				open.emplace_back(n);
			}
			index++;
		}
		return false;
	}

protected:
	void setEdgeStatus(unsigned int a, unsigned int b, Edge::CollisionCheckingStatus status) {
		auto vertexAndEdges = edges.find(a);
		if(vertexAndEdges == edges.end()) {
			return;
		}

		auto &edges = vertexAndEdges->second;

		auto edge = edges.find(b);
		if(edge == edges.end()) {
			return;
		}

		edge->second.edgeStatus = status;
	}

	void setEdgeStatusUnchecked(unsigned int a, unsigned int b, Edge::CollisionCheckingStatus status) {
		edges[a][b].edgeStatus = status;
	}

	std::vector<Vertex *> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	const ompl::base::MotionValidatorPtr &motionValidator;
	const ompl::base::State *start, *goal;
};
