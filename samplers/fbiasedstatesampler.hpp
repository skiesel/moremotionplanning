#pragma once

#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <thread>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalState.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLStateValidityChecker.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class FBiasedStateSampler : public ompl::base::UniformValidStateSampler {

protected:
	struct Vertex {
		Vertex(unsigned int id) : id(id), g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()),
		pdfID(0), state(NULL), extraData(NULL) {}

		unsigned int id;
		double g, h, f;
		double score;
		unsigned int pdfID;
		ompl::base::State *state;

		void *extraData;
	};

	struct VertexWrapper {
		VertexWrapper(Vertex *vertex) : vertex(vertex), onOpen(false), onClosed(false), currentParent(NULL) {}

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
		virtual bool sort(const VertexWrapper*) const = 0;

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
		bool onOpen, onClosed;
		unsigned int heapIndex;

		struct Parent {
			Parent(unsigned int parent, double cost) : parent(parent), cost(cost) {}
			static bool HeapCompare(const Parent *r1, const Parent *r2) {
				return r1->cost < r2->cost;
			}
			unsigned int parent;
			double cost;
		};
		
		bool addParent(unsigned int parent, double cost) {
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

		std::vector<Parent*> parents;
		Parent *currentParent;
	};

	struct VertexGWrapper : public VertexWrapper {
		VertexGWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return vertex->g;
		}
		void setVal(double c) {
			vertex->g = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return this->vertex->g < n2->vertex->g;
		}
		bool operator<(const VertexWrapper *n2) const {
			return this->vertex->g < n2->vertex->g;
		}
	};

	struct VertexHWrapper : public VertexWrapper {
		VertexHWrapper(Vertex *vertex) : VertexWrapper(vertex) {}

		double getVal() const {
			return vertex->h;
		}
		void setVal(double c) {
			vertex->h = c;
		}
		bool sort(const VertexWrapper *n2) const {
			return this->vertex->h < n2->vertex->h;
		}
		bool operator<(const VertexWrapper *n2) const {
			return this->vertex->h < n2->vertex->h;
		}
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

	const base::State *stateIsAlreadyGeometric(const base::State *state, unsigned int /*index*/) const {
		return state;
	}

public:
	double abstractDistanceFunction(const Vertex *a, const Vertex *b) const {
		assert(a->state != NULL);
		assert(b->state != NULL);
		return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(a->state, b->state);
	}

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

	FBiasedStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                    const FileMap &params, bool addAllRegionsToPDF = true, bool generateAllRegionScores = true) : UniformValidStateSampler(base), 
		fullStateSampler(base->allocStateSampler()), addAllRegionsToPDF(addAllRegionsToPDF), generateAllRegionScores(generateAllRegionScores),
		prmSize(params.integerVal("PRMSize")), numEdges(params.integerVal("NumEdges")), stateRadius(params.doubleVal("StateRadius")),
		motionValidator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->getMotionValidator()) {
			omega = params.exists("Omega") ? params.doubleVal("Omega") : 1.0;
		}

	virtual ~FBiasedStateSampler() {
		for(auto vert : vertices) {
			delete vert;
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

			nn->setDistanceFunction(boost::bind(&FBiasedStateSampler::abstractDistanceFunction, this, _1, _2));

			generateVertices(abstractStart, abstractGoal, prmSize);
			generateEdges(numEdges);

			std::vector<VertexWrapper*> wrappers;
			wrappers.reserve(vertices.size());
			std::vector<VertexHWrapper> hWrappers;
			hWrappers.reserve(vertices.size());
			for(auto v : vertices) {
				hWrappers.emplace_back(v);
				wrappers.emplace_back(&hWrappers.back());
			}

			dijkstra(wrappers[1], wrappers);

			std::vector<VertexGWrapper> gWrappers;
			gWrappers.reserve(vertices.size());
			unsigned int i = 0;
			for(auto v : vertices) {
				gWrappers.emplace_back(v);
				wrappers[i++] = &gWrappers.back();
			}

			dijkstra(wrappers[0], wrappers);

			connected = !(std::isinf(vertices[0]->g) || std::isinf(vertices[0]->h) || std::isinf(vertices[1]->g) || std::isinf(vertices[1]->h));

			if(!connected) {
				OMPL_INFORM("not connected! recomputing...");
				for(auto vert : vertices) {
					delete vert;
				}
				edges.clear();
				prmSize *= 1.5;
			} else {
				
			}
		} while(!connected);

		if(generateAllRegionScores) {
			generateRegionScores();
		}

		if(addAllRegionsToPDF) {
			for(unsigned int i = 0; i < vertices.size(); ++i) {
				auto el = pdf.add(vertices[i], vertices[i]->score);
				vertices[i]->pdfID = el->getId();
			}
		}
#ifdef STREAM_GRAPHICS
		generatePythonPlotting();
#endif
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

	void streamVisualization() const {
		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			if(vertices[i]->score < min) min = vertices[i]->score;
			if(vertices[i]->score > max) max = vertices[i]->score;
		}

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto state = vertices[i]->state->as<ompl::base::SE3StateSpace::StateType>();
			auto color = getColor(min, max, vertices[i]->score);

			streamPoint(state, color[0], color[1], color[2], 1);
		}
	}

	void generatePythonPlotting() const {
		FILE* f = fopen("prm.py", "w");

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			if(vertices[i]->score < min) min = vertices[i]->score;
			if(vertices[i]->score > max) max = vertices[i]->score;
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
					auto color = getColor(min, max, vertices[i]->score);
					fprintf(f, (i < vertices.size()-1) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
				}
			}
			fprintf(f, (coord < 3) ? "]," : "], depthshade=False");
		}
		fprintf(f, ")\nplt.show()\n");
		fclose(f);
	}

	virtual bool sample(ompl::base::State *state) {
		Vertex *randomVertex = pdf.sample();

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		vertexState = randomVertex->state;

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("FBiasedStateSampler::sampleNear", "not implemented");
		return false;
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

	virtual void generateEdges(unsigned int howManyConnections) {
		Timer timer("Edge Generation");
		auto distanceFunc = nn->getDistanceFunction();

		for(Vertex* vertex : vertices) {
			edges[vertex->id];

			std::vector<Vertex *> neighbors;
			nn->nearestK(vertex, howManyConnections+1, neighbors);

			assert(howManyConnections+1 >= neighbors.size());

			for(Vertex* neighbor : neighbors) {
				double distance = distanceFunc(vertex, neighbor);
				if(distance == 0) continue;

				edges[vertex->id][neighbor->id] = Edge(neighbor->id, distance);
			}
		}
	}

	// virtual void generateEdges(unsigned int howManyConnections) {
	// 	Timer timer("Edge Generation");
	// 	auto distanceFunc = nn->getDistanceFunction();

	// 	std::vector<Vertex *> neighbors;
	// 	for(auto vertex : vertices) {
	// 		edges[vertex->id];

	// 		neighbors.clear();
	// 		nn->nearestK(vertex, howManyConnections+1, neighbors);

	// 		assert(howManyConnections+1 >= neighbors.size());

	// 		for(auto neighbor : neighbors) {
	// 			double distance = distanceFunc(vertex, neighbor);
	// 			if(distance == 0) continue;

	// 			if(motionValidator->checkMotion(vertex->state, neighbor->state)) {
	// 				edges[vertex->id][neighbor->id] = Edge(neighbor->id, distance);
	// 				edges[neighbor->id][vertex->id] = Edge(vertex->id, distance);
	// 			}
	// 		}
	// 	}
	// }


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

	virtual void dijkstra(VertexWrapper *start, const std::vector<VertexWrapper*> &wrappers) {
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

				double newValue = current->getVal() + getEdgeCostBetweenCells(current->getId(), kidIndex);
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

	// void dijkstra(VertexWrapper *start, const std::vector<VertexWrapper*> &wrappers) {
	// 	Timer t("dijkstra");
	// 	InPlaceBinaryHeap<VertexWrapper, VertexWrapper> open;
	// 	std::unordered_set<unsigned int> closed;
	// 	start->setVal(0);
	// 	open.push(start);

	// 	closed.insert(start->getId());

	// 	while(!open.isEmpty()) {
	// 		VertexWrapper *current = open.pop();

	// 		closed.insert(current->getId());

	// 		std::vector<unsigned int> kids = getNeighboringCells(current->getId());
	// 		for(unsigned int kidIndex : kids) {
	// 			if(closed.find(kidIndex) != closed.end()) continue;

	// 			double newValue = current->getVal() + getEdgeCostBetweenCells(current->getId(), kidIndex);
	// 			VertexWrapper *kid = wrappers[kidIndex];

	// 			if(newValue < kid->getVal()) {
	// 				kid->setVal(newValue);

	// 				if(open.inHeap(kid)) {
	// 					open.siftFromItem(kid);
	// 				} else {
	// 					open.push(kid);
	// 				}
	// 			}
	// 		}
	// 	}
	// }

	void generateRegionScores() {
		double minF = std::numeric_limits<double>::infinity();
		std::vector<Vertex *> untouched;
		for(Vertex *n : vertices) {
			n->f = n->g + n->h;
			if(n->f >= 0) {
				if(n->f < minF) {
					minF = n->f;
				}
			} else {
				untouched.emplace_back(n);
			}
		}

		double numerator = pow(minF, omega);
		double minScore = std::numeric_limits<double>::infinity();
		double scoreSum = 0;
		for(Vertex *n : vertices) {
			if(n->f >= 0) {
				n->score = numerator / pow(n->f, omega);
				scoreSum += n->score;
				if(n->score <= minScore) {
					minScore = n->score;
				}
			}
		}

		minScore /= 2;
		for(Vertex *n : untouched) {
			n->score = minScore;
		}
	}

	

	StateSamplerPtr fullStateSampler;
	boost::shared_ptr< NearestNeighbors<Vertex *> > nn;
	std::vector<Vertex *> vertices;
	std::unordered_map<unsigned int, std::unordered_map<unsigned int, Edge>> edges;
	ProbabilityDensityFunction<Vertex> pdf;
	bool addAllRegionsToPDF, generateAllRegionScores;
	unsigned int prmSize, numEdges;
	double omega, stateRadius;
	ompl::base::MotionValidatorPtr motionValidator;
};

}

}
