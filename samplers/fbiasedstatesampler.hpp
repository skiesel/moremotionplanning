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

class FBiasedStateSampler : public ompl::base::UniformValidStateSampler {

protected:
	struct Vertex {
		Vertex(unsigned int id) : id(id), g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()), pdfID(0), state(NULL), extraData(NULL) {}

		~Vertex() {}

		static double getG(const Vertex *n) {
			return n->g;
		}
		static void setG(Vertex *n, double c) {
			n->g = c;
		}
		static bool sortG(const Vertex *n1, const Vertex *n2) {
			return n1->g < n2->g;
		}

		static double getH(const Vertex *n) {
			return n->h;
		}
		static void setH(Vertex *n, double c) {
			n->h = c;
		}
		static bool sortH(const Vertex *n1, const Vertex *n2) {
			return n1->h < n2->h;
		}

		unsigned int heapIndex;
		static bool pred(const Vertex *a, const Vertex *b) {
			return sort(a,b);
		}
		static unsigned int getHeapIndex(const Vertex *r) {
			return r->heapIndex;
		}
		static void setHeapIndex(Vertex *r, unsigned int i) {
			r->heapIndex = i;
		}

		static std::function<bool(const Vertex *, const Vertex *)> sort;
		unsigned int id;
		double g, h, f;
		double score;
		unsigned int pdfID;
		ompl::base::State *state;

		void *extraData;
	};

	struct Edge {
		Edge() {}

		Edge(unsigned int endpoint, double weight) : endpoint(endpoint), weight(weight) {

		}

		std::size_t operator()(const Edge &e) const {
			return e.endpoint;
		}

		bool operator==(const Edge &e) const {
			return e.endpoint == endpoint;
		}

		unsigned int endpoint;
		double weight;
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
	FBiasedStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	                    double omega, double stateRadius, bool addAllRegionsToPDF = true) : UniformValidStateSampler(base), 
	fullStateSampler(base->allocStateSampler()), omega(omega), stateRadius(stateRadius) {

		//Stolen from tools::SelfConfig::getDefaultNearestNeighbors
		if(base->getStateSpace()->isMetricSpace()) {
			// if (specs.multithreaded)
			//  nn.reset(new NearestNeighborsGNAT<Vertex*>());
			//else
			nn.reset(new NearestNeighborsGNATNoThreadSafety<Vertex *>());
		} else {
			nn.reset(new NearestNeighborsSqrtApprox<Vertex *>());
		}

		nn->setDistanceFunction(boost::bind(&FBiasedStateSampler::abstractDistanceFunction, this, _1, _2));

		generateVertices(10000);
		generateEdges(10);

		Vertex startVertex(0);
		startVertex.state = start;
		dijkstraG(nn->nearest(&startVertex));

		unsigned int bestId = 0;
		double minDistance = std::numeric_limits<double>::infinity();
		double distance;
		for(auto vertex : vertices) {
			ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
			vertexState = vertex->state;
			ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

			goal->isSatisfied(fullState.get(), &distance);
			if(distance <  minDistance) {
				bestId = vertex->id;
				minDistance = distance;
			}
		}
		dijkstraH(vertices[bestId]);

		generateRegionScores();

		if(addAllRegionsToPDF) {
			for(unsigned int i = 0; i < vertices.size(); ++i) {
				auto el = pdf.add(vertices[i], vertices[i]->score);
				vertices[i]->pdfID = el->getId();
			}
		}

		// dumpToStderr();
	}

	virtual ~FBiasedStateSampler() {}

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
			if(vertices[i]->score < min) min = vertices[i]->score;
			if(vertices[i]->score > max) max = vertices[i]->score;
		}

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			auto state = vertices[i]->state->as<ompl::base::SE3StateSpace::StateType>();
			auto color = getColor(min, max, vertices[i]->score);

			fprintf(stderr, "point %g %g %g %g %g %g 1\n", state->getX(), state->getY(), state->getZ(), color[0], color[1], color[2]);
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
		// fullStateSampler->sampleUniform(state);

		return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("FBiasedStateSampler::sampleNear", "not implemented");
		return false;
	}

protected:
	void generateVertices(unsigned int howMany) {
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();

		vertices.resize(howMany);

		for(unsigned int i = 0; i < howMany; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}
	}

	void generateEdges(unsigned int howManyConnections) {
		ompl::base::MotionValidatorPtr motionValidator = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->getMotionValidator();;

		auto distanceFunc = nn->getDistanceFunction();

		std::vector<Vertex *> neighbors;
		for(auto vertex : vertices) {
			neighbors.clear();
			nn->nearestK(vertex, howManyConnections+1, neighbors);

			assert(howManyConnections+1 >= neighbors.size());

			for(auto neighbor : neighbors) {

				if(edges.find(vertex->id) != edges.end() && 
					edges[vertex->id].find(neighbor->id) != edges[vertex->id].end()) {
					continue;
				}

				double distance = distanceFunc(vertex, neighbor);
				if(distance == 0) continue;

				if(motionValidator->checkMotion(vertex->state, neighbor->state)) {
					edges[vertex->id][neighbor->id] = Edge(neighbor->id, distance);
					edges[neighbor->id][vertex->id] = Edge(vertex->id, distance);
				}
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

	void dijkstraG(Vertex *start) {
		Vertex::sort = Vertex::sortG;
		dijkstra(start, Vertex::getG, Vertex::setG);
	}

	void dijkstraH(Vertex *start) {
		Vertex::sort = Vertex::sortH;
		dijkstra(start, Vertex::getH, Vertex::setH);
	}

	void dijkstra(Vertex *start,
	              std::function<double(const Vertex *)> peek,
	              std::function<void(Vertex *, double)> update) {

		InPlaceBinaryHeap<Vertex, Vertex> open;
		std::unordered_set<unsigned int> closed;
		update(start, 0);
		open.push(start);

		closed.insert(start->id);

		while(!open.isEmpty()) {
			Vertex *current = open.pop();

			closed.insert(current->id);

			std::vector<unsigned int> kids = getNeighboringCells(current->id);
			for(unsigned int kidIndex : kids) {
				if(closed.find(kidIndex) != closed.end()) continue;

				double newValue = peek(current) + getEdgeCostBetweenCells(current->id, kidIndex);
				Vertex *kid = vertices[kidIndex];

				if(newValue < peek(kid)) {
					update(kid, newValue);

					if(open.inHeap(kid)) {
						open.siftFromItem(kid);
					} else {
						open.push(kid);
					}
				}
			}
		}
	}

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
				untouched.push_back(n);
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
	double omega, stateRadius;
};

std::function<bool(const typename FBiasedStateSampler::Vertex *, const typename FBiasedStateSampler::Vertex *)> FBiasedStateSampler::Vertex::sort = FBiasedStateSampler::Vertex::sortG;
}

}
