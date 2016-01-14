#pragma once

#include <unordered_set>
#include <unordered_map>

#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLStateValidityChecker.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class PlakuStateSampler : public ompl::base::UniformValidStateSampler {

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
			Parent* newParent = new Parent(parent, cost, path);
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

		std::vector<Parent*> parents;
		Parent *currentParent;
	};

	struct Edge {
		enum CollisionCheckingStatus {
			UNKNOWN = 0,
			INVALID = 1,
			VALID = 2,
		};

		Edge() {}

		Edge(unsigned int endpoint, double weight) : endpoint(endpoint), weight(weight), edgeStatus(UNKNOWN) {}

		std::size_t operator()(const Edge &e) const {
			return e.endpoint;
		}

		bool operator==(const Edge &e) const {
			return e.endpoint == endpoint;
		}

		unsigned int endpoint;
		double weight;
		unsigned int edgeStatus;
	};

	double abstractDistanceFunction(const Vertex *a, const Vertex *b) const {
		assert(a->state != NULL);
		assert(b->state != NULL);
		return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(a->state, b->state);
	}

	const base::State *stateIsAlreadyGeometric(const base::State *state, unsigned int /*index*/) const {
		return state;
	}

public:
	struct Timer {
		Timer(const std::string &print) : print(print) {
			OMPL_INFORM("starting : %s", print.c_str());
			start = clock();

		}
		~Timer() {
			OMPL_INFORM("ending : %s : \t%g", print.c_str(), (double)(clock()-start) / CLOCKS_PER_SEC);
		}
		clock_t start;
		std::string print;
	};

	PlakuStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start_, const ompl::base::GoalPtr &goal, unsigned int prmSize, unsigned int numEdges, double alpha, double b, double stateRadius)
		: UniformValidStateSampler(base), fullStateSampler(base->allocStateSampler()), alpha(alpha), b(b), stateRadius(stateRadius), activeRegion(NULL),
		motionValidator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->getMotionValidator()) {

		base->getStateSpace()->copyState(start, start_);
	}

	virtual ~PlakuStateSampler() {
		for(auto vertex : vertices) {
			delete vertex;
		}
	}

	virtual void initialize() {
		Timer timer("Abstraction Computation");

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

			nn->setDistanceFunction(boost::bind(&PlakuStateSampler::abstractDistanceFunction, this, _1, _2));

			generateVertices(abstractStart, abstractGoal, prmSize);
			generateEdges(numEdges);


		    startRegionId = 0;
	    	goalRegionId = 1;
			dijkstra(vertices[goalRegionId]);

			connected = !std::isinf(vertices[startRegionId]->heuristic);

			if(!connected) {
				OMPL_WARN("not connected! recomputing...");
				for(auto vert : vertices) {
					delete vert;
				}
				edges.clear();
				prmSize *= 1.5;
			}
		} while(!connected);

		//Believe it or not but we can actually generate a vertex
		//that can not be connected to its nearest neighbors
		//if that happens, we need to remove it because it we
		//generate an edge near it, and it's selected, we'll
		//end up with a segfault because it has no region path
		for(auto v : vertices) {
			if(std::isinf(v->heuristic)) {
				nn->remove(v);
			}
		}

		// generatePythonPlotting();

		reached(start);
	}

	std::vector<double> getColor(double min, double max, double value) const {
		std::vector<double> color(3);

		value = ((value - min) / (max - min)) * 765;

		if(value < 255) {
			color[0] = 0;
			color[1] = value / 2;
			color[2] = 255 - value;
		} else if(value < 510) {
			double relVal = value - 255;
			color[0] = relVal;
			color[1] = (relVal + 255) / 2;
			color[2] = 0;
		} else {
			double relVal = value - 510;
			color[0] = 255;
			color[1] = 255 - relVal;
			color[2] = 0;
		}

		for(unsigned int i = 0; i < 3; ++i) {
			color[i] /= 255;
		}

		return color;
	}

	void dumpToStderr() const {
		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			if(vertices[i]->weight < min) min = vertices[i]->weight;
			if(vertices[i]->weight > max) max = vertices[i]->weight;
		}

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto state = vertices[i]->state->as<ompl::base::SE3StateSpace::StateType>();
			auto color = getColor(min, max, vertices[i]->weight);

			fprintf(stderr, "point %g %g %g %g %g %g 1\n", state->getX(), state->getY(), state->getZ(), color[0], color[1], color[2]);
		}
	}

	void generatePythonPlotting() const {
		FILE *f = fopen("prm.py", "w");

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			if(vertices[i]->heuristic < min) min = vertices[i]->heuristic;
			if(vertices[i]->heuristic > max) max = vertices[i]->heuristic;
		}
		fprintf(f, "import numpy as np\nfrom mpl_toolkits.mplot3d import Axes3D\nimport matplotlib.pyplot as plt\n");
		fprintf(f, "fig = plt.figure()\nax = fig.add_subplot(111, projection='3d')\nax.scatter(");

		for(unsigned int coord = 0; coord < 4; ++coord) {
			if(coord == 3) {
				fprintf(f, "c=");
			}
			fprintf(f, "[");
			for(unsigned int i = 0; i < vertices.size(); ++i) {
				auto state = vertices[i]->state->as<ompl::base::SE3StateSpace::StateType>();
				if(coord == 0) {
					fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getX());
				} else if(coord == 1) {
					fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getY());
				} else if(coord == 2) {
					fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getZ());
				} else if(coord == 3) {
					auto color = getColor(min, max, vertices[i]->heuristic);
					fprintf(f, (i < vertices.size()-1) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
				}
			}
			fprintf(f, (coord < 3) ? "]," : "], depthshade=False");
		}
		fprintf(f, ")\nplt.show()\n");
		fclose(f);
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
			vertexState = vertices[regionAlongPath]->state;
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
		Vertex v(0);
		auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
		v.state = ss.get();
		unsigned int newCellId = nn->nearest(&v)->id;
		if(!vertices[newCellId]->onOpen) {
			vertices[newCellId]->selected(alpha);
			regionHeap.push_back(vertices[newCellId]);
			std::push_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
			vertices[newCellId]->onOpen = true;
		}
	}

protected:
	virtual void generateVertices(const ompl::base::State *start, const ompl::base::State *goal, unsigned int howMany) {
		Timer timer("Vertex Generation");
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();

		vertices.resize(howMany);

		vertices[0] = new Vertex(0);
		vertices[0]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[0]->state, start);
		nn->add(vertices[0]);

		vertices[1] = new Vertex(1);
		vertices[1]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[1]->state, goal);
		nn->add(vertices[1]);

		for(unsigned int i = 2; i < howMany; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}
	}

	void setEdgeStatus(unsigned int a, unsigned int b, unsigned int status) {
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

	bool edgeExists(unsigned int a, unsigned int b) const {
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
			if(motionValidator->checkMotion(vertices[a]->state, vertices[b]->state)) {
				edge->second.edgeStatus = Edge::VALID;
				setEdgeStatus(b, a, Edge::VALID);
			} else {
				edge->second.edgeStatus = Edge::INVALID;
				setEdgeStatus(b, a, Edge::INVALID);
			}
		}

		return edge->second.edgeStatus == Edge::VALID;
	}

	void generateEdges(unsigned int howManyConnections) {
		Timer timer("Edge Generation");
		auto distanceFunc = nn->getDistanceFunction();

		for(Vertex* vertex : vertices) {
			edges[vertex->id];

			std::vector<Vertex *> neighbors;
			nn->nearestK(vertex, howManyConnections+1, neighbors);

			assert(howManyConnections+1 >= neighbors.size());

			for(Vertex* neighbor : neighbors) {
				if(edgeExists(vertex->id, neighbor->id)) continue;

				double distance = distanceFunc(vertex, neighbor);
				if(distance == 0) continue;

				edges[vertex->id][neighbor->id] = Edge(neighbor->id, distance);
				edges[neighbor->id][vertex->id] = Edge(vertex->id, distance);
			}
		}
	}

	std::vector<unsigned int> getNeighboringCells(unsigned int index) const {
		auto vertexAndEdges = edges.find(index);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		std::vector<unsigned int> ids;
		ids.reserve(edges.size());
		for(const auto &edge : edges) {
			ids.push_back(edge.second.endpoint);
		}

		return ids;
	}

	double getEdgeCostBetweenCells(unsigned int c1, unsigned int c2) const {
		auto vertexAndEdges = edges.find(c1);
		assert(vertexAndEdges != edges.end());

		const auto &edges = vertexAndEdges->second;

		auto edge = edges.find(c2);
		assert(edge != edges.end());

		return edge->second.weight;
	}

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
				if(!isValidEdge(parentIndex, current->id)) {
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

				double newValue = current->heuristic + getEdgeCostBetweenCells(current->id, kidIndex);

				Vertex *kid = vertices[kidIndex];

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

	StateSamplerPtr fullStateSampler;
	boost::shared_ptr< NearestNeighbors<Vertex *> > nn;
	std::vector<Vertex *> vertices, regionHeap;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	ompl::RNG randomNumbers;
	ompl::base::MotionValidatorPtr motionValidator;

	ompl::base::State *start;
	unsigned int startRegionId, goalRegionId, prmSize, numEdges;
	Vertex *activeRegion;

	double alpha, b, stateRadius;
};

}

}
