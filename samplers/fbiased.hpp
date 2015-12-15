#include <unordered_set>
#include <unordered_map>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include "../domains/geometry/detail/FCLContinuousMotionValidator.hpp"
#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

  class FBiasedStateSampler : public ompl::base::ValidStateSampler {

  struct Vertex {
    Vertex(unsigned int id) : id(id), g(std::numeric_limits<double>::infinity()), h(std::numeric_limits<double>::infinity()), touched(false), pdfID(0) {}

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
    bool touched;
    unsigned int pdfID;
    ompl::base::State *state;
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

  public:
    FBiasedStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal) : ValidStateSampler(base),
    fullStateSampler(base->allocStateSampler()) {
      omega = 8;
      stateRadius = 1;

      ompl::base::StateSpacePtr abstractSpace = globalAppBaseControl->getGeometricComponentStateSpace();
      ompl::base::StateSamplerPtr abstractSampler = si_->getStateSpace()->allocSubspaceStateSampler(abstractSpace);


      ompl::base::SpaceInformationPtr baseSpaceInfoPtr(base);
      ompl::app::FCLContinuousMotionValidator motionValidator(baseSpaceInfoPtr, ompl::app::Motion_3D);

      generateVertices(10000, abstractSpace, abstractSampler);
      generateEdges(10, motionValidator);

      Vertex startVertex(0);
      startVertex.state = start; 
      dijkstraG(nn->nearest(&startVertex));


      unsigned int bestId = 0;
      double minDistance = std::numeric_limits<double>::infinity();
      double distance;
      for(auto vertex : vertices) {
        ompl::base::ScopedState<> vertexState(globalAppBaseControl->getGeometricComponentStateSpace());
        vertexState = vertex->state;
        ompl::base::ScopedState<> fullState = globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

        goal->isSatisfied(fullState.get(), &distance);
        if(distance <  minDistance) {
          bestId = vertex->id;
          minDistance = distance;
        }
      }

      dijkstraH(vertices[bestId]);

      generatePDF();
    }

    virtual ~FBiasedStateSampler() {}

    virtual bool sample(ompl::base::State *state) {
      Vertex *randomVertex = pdf.sample();

      ompl::base::ScopedState<> vertexState(globalAppBaseControl->getGeometricComponentStateSpace());
      vertexState = randomVertex->state;

      ompl::base::ScopedState<> fullState = globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

      fullStateSampler->sampleUniformNear(fullState.get(), state, stateRadius);
      return true;
    }

    virtual bool sampleNear(ompl::base::State*, const ompl::base::State*, const double) {
      throw ompl::Exception("FBiasedStateSampler::sampleNear", "not implemented");
      return false;
    }

  template<template<typename T> class NN>
  void setNearestNeighbors() {
    nn.reset(new NN<Vertex *>());
  }

  private:
    void generateVertices(unsigned int howMany, const ompl::base::StateSpacePtr &abstractSpace, const ompl::base::StateSamplerPtr &abstractSampler) {

      vertices.resize(howMany);
      for(auto vertex : vertices) {
        vertex->state = abstractSpace->allocState();
        abstractSampler->sampleUniform(vertex->state);
        nn->add(vertex);
      }
    }

    void generateEdges(unsigned int howManyConnections, const ompl::app::FCLContinuousMotionValidator &motionValidator) {
      auto distanceFunc = nn->getDistanceFunction();

      std::vector<Vertex*> neighbors;
      for(auto vertex : vertices) {
        nn->nearestK(vertex, howManyConnections+1, neighbors);
        for(auto neighbor : neighbors) {
          double distance = distanceFunc(vertex, neighbor);
          if(distance == 0) continue;

          if(motionValidator.checkMotion(vertex->state, neighbor->state)) {
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

    void generatePDF() {
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

      
      for(unsigned int i = 0; i < vertices.size(); ++i) {
        auto el = pdf.add(vertices[i], vertices[i]->score);
        vertices[i]->pdfID = el->getId();
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
