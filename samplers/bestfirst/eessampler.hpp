#pragma once

#include "../bestfirstsampler.hpp"
#include "../structs/rbtree.hpp"

namespace ompl {

namespace base {

class EESSampler : public ompl::base::BestFirstSampler {

  struct ExtraData {
    ExtraData(double clearance) : clearance(clearance), fhat(std::numeric_limits<double>::infinity()),
    e(std::numeric_limits<double>::infinity()), ehat(std::numeric_limits<double>::infinity()),
    fIndex(std::numeric_limits<unsigned int>::max()), eHatIndex(std::numeric_limits<unsigned int>::max()) {}

    double clearance, fhat, e, ehat;
    unsigned int fIndex, eHatIndex;
  };

  static inline ExtraData* extraData(Vertex *v) {
    return ((ExtraData*)v->extraData);
  }

  static inline const ExtraData* extraData(const Vertex *v) {
    return ((ExtraData*)v->extraData);
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

  struct SortOnF {
    static unsigned int getHeapIndex(const Vertex *n) {
      return extraData(n)->fIndex;
    }

    static void setHeapIndex(Vertex *n, unsigned int index) {
      extraData(n)->fIndex = index;
    }

    static bool pred(const Vertex *a, const Vertex *b) {
      if(a->f == b->f) {
        return a->g > b->g;
      }
      return a->f < b->f;
    }
  };

  struct SortOnEHat {
    static unsigned int getHeapIndex(const Vertex *n) {
      return extraData(n)->eHatIndex;
    }

    static void setHeapIndex(Vertex *n, unsigned int index) {
      extraData(n)->eHatIndex = index;
    }

    static bool pred(const Vertex *a, const Vertex *b) {
      if(extraData(a)->ehat == extraData(b)->ehat) {
        return extraData(a)->fhat < extraData(b)->fhat;
      }
      return extraData(a)->ehat < extraData(b)->ehat;
    }
  };

  struct SortOnFHat {
    static int compare(const Vertex *a, const Vertex *b) {
      if(extraData(a)->fhat == extraData(b)->fhat) {
        return a->f - b->f;
      }
      return extraData(a)->fhat - extraData(b)->fhat;
    }
  };

  class AbstractEffortEstimator : public ompl::base::StateCostIntegralObjective {
  public:
      AbstractEffortEstimator(const ompl::base::SpaceInformationPtr& si) :
          ompl::base::StateCostIntegralObjective(si, false) {}

      ompl::base::Cost stateCost(const ompl::base::State* s) const {
          return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
      }
  };

public:

  EESSampler( ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    ) :
    FBiasedStateSampler(base, start, goal, prmSize, numEdges, omega, stateRadius, false),
    effortEstimator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()) {
      weight = params.exists("Weight") ? params.doubleVal("Weight") : 1.0;

    }

  virtual ~EESSampler() {}

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

      useEdgeEffort = false;
      dijkstra(wrappers[1], wrappers);

      std::vector<VertexEWrapper> eWrappers;
      eWrappers.reserve(vertices.size());
      unsigned int i = 0;
      for(auto v : vertices) {
        eWrappers.emplace_back(v);
        wrappers[i++] = &eWrappers.back();
      }

      useEdgeEffort = true;
      dijkstra(wrappers[1], wrappers);

      connected = !(std::isinf(extraData(vertices[0])->e) || std::isinf(vertices[0]->h));

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

    auto neighbors = getNeighboringCells(0);
    for(auto n : neighbors) {
      vertices[n]->g = vertices[0]->g + getEdgeCostBetweenCells(0, n);
      vertices[n]->f = vertices[n]->g + vertices[n]->h;
      extraData(vertices[n])->fhat = vertices[n]->f;
      extraData(vertices[n])->ehat = extraData(vertices[n])->e;
      
      addToQueues(vertices[n]);
    }

#ifdef STREAM_GRAPHICS
    generatePythonPlotting([] (const Vertex* v) { return v->h; }, "prm1.py");
    generatePythonPlotting([] (const Vertex* v) { return std::log10(extraData(v)->e);}, "prm2.py");
#endif

  }

  void generatePythonPlotting(std::function<double(const Vertex*)> getValue, const std::string fname) const {
    FILE* f = fopen(fname.c_str(), "w");

    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    for(unsigned int i = 0; i < vertices.size(); ++i) {
      double value = getValue(vertices[i]);
      if(std::isinf(value) || std::isnan(value)) continue;

      if(value < min) min = value;
      if(value > max) max = value;
    }

    fprintf(stderr, "%g -> %g\n", min, max);

    fprintf(f, "import numpy as np\nfrom mpl_toolkits.mplot3d import Axes3D\nimport matplotlib.pyplot as plt\n");
    fprintf(f, "fig = plt.figure()\nax = fig.add_subplot(111, projection='3d')\nax.scatter(");

    for(unsigned int coord = 0; coord < 4; ++coord) {
      if(coord == 3) {
        fprintf(f, "c=");
      }
      fprintf(f, "[");
      for(unsigned int i = 0; i < vertices.size(); ++i) {
        double value = getValue(vertices[i]);
        if(std::isinf(value) || std::isnan(value)) continue;

        auto state = vertices[i]->state->as<ompl::base::SE3StateSpace::StateType>();
        if(coord == 0) {
          fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getX());
        } else if(coord == 1) {
          fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getY());
        } else if(coord == 2) {
          fprintf(f, (i < vertices.size()-1) ? "%g, ": "%g", state->getZ());
        } else if(coord == 3) {
          auto color = getColor(min, max, value);
          fprintf(f, (i < vertices.size()-1) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
        }
      }
      fprintf(f, (coord < 3) ? "]," : "], depthshade=False");
    }
    fprintf(f, ")\nplt.show()\n");
    fclose(f);
  }

  virtual bool sample(ompl::base::State *state) {
    if(randomNumbers.uniform01() <= 0.1) {
      fullStateSampler->sampleUniform(state);
      return true;
    }

    oldBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;
    lastVertex = selectVertex();
    //Vertex* n = selectVertex_con();
    //Vertex* n = selectVertex_opt();

    ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
    vertexState = lastVertex->state;

    ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

    fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

    return true;
  }

  virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
    throw ompl::Exception("EESSampler::sampleNear", "not implemented");
    return false;
  }

  virtual void reached(ompl::base::State *fromState, ompl::base::State *toState) {
    ompl::base::ScopedState<> incomingState(si_->getStateSpace());
    incomingState = fromState;

    Vertex v(0);
    auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
    v.state = ss.get();
    Vertex *fromVertex = nn->nearest(&v);

    incomingState = toState;
    ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
    v.state = ss.get();

    Vertex *reachedVertex = nn->nearest(&v);

//     if(!extraData(vertex)->expanded) {
//       extraData(vertex)->expanded = true;
//       auto neighbors = getNeighboringCells(vertex->id);
//       for(auto n : neighbors) {
//         double newG = vertex->g + getEdgeCostBetweenCells(vertex->id, n);
        
// //check the queues to see if we need updating

//         if(newG < vertices[n]->g) {
//           vertices[n]->g = vertex->g + getEdgeCostBetweenCells(vertex->id, n);
//           vertices[n]->f = vertices[n]->g + vertices[n]->h;
//           extraData(vertices[n])->fhat = vertices[n]->f;
//           extraData(vertices[n])->ehat = extraData(vertices[n])->e;
//         }

//         // addToQueues(vertices[n]);
//       }
//     }

//     if(vertex == lastVertex) {
//       //reached and expanded above
//     } else {
//       //didn't make it
//       extraData(lastVertex)->e *= 1.1;
//       addToQueues(lastVertex);
//     }

//     double newBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;

//     if(oldBestFHatVal != newBestFHatVal) {
//       rebuildFocal();
//     }

//     lastVertex = NULL;
  }

protected:
  virtual void generateVertices(const ompl::base::State *start, const ompl::base::State *goal, unsigned int howMany) {
    FBiasedStateSampler::generateVertices(start, goal, howMany);

    for(auto v : vertices) {
      v->extraData = new ExtraData(effortEstimator.stateCost(v->state).value());
    }
  }

  Vertex* getBestFAndSyncQueues() {
    Vertex* best = cleanup.pop();
    open.remove(best);
    if(focal.inHeap(best)) {
      focal.remove(best);
    }
    return best;
  }

  Vertex* getBestFHatAndSyncQueues() {
    Vertex* best = open.extractLeftmost();
    if(focal.inHeap(best))
      focal.remove(best);
    cleanup.remove(best);
    return best;
  }

  Vertex* getBestDHatAndSyncQueues() {
    Vertex* best = focal.pop();
    open.remove(best);
    cleanup.remove(best);
    return best;
  }

  Vertex* selectVertex() {
    const Vertex* bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
    const Vertex* bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
    const Vertex* bestE = focal.isEmpty() ? NULL : focal.peek();
    if(extraData(bestE)->fhat <= weight * bestF->f) {
      return getBestDHatAndSyncQueues();
    } else if(extraData(bestFhat)->fhat <= weight * bestF->f) {
      return getBestFHatAndSyncQueues();
    } else {
      return getBestFAndSyncQueues();
    }
  }

  Vertex* selectVertex_con() {
    
    const Vertex* bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
    const Vertex* bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
    const Vertex* bestE = focal.isEmpty() ? NULL : focal.peek();

    if(extraData(bestFhat)->fhat > weight * bestF->f) {
      return getBestFAndSyncQueues();
    } else if(extraData(bestE)->fhat <= weight * bestF->f) {
      return getBestDHatAndSyncQueues();
    } else {
      return getBestFHatAndSyncQueues();
    }
  }

  Vertex* selectVertex_opt() {
    
    const Vertex* bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
    const Vertex* bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
    const Vertex* bestE = focal.isEmpty() ? NULL : focal.peek();

    if(incumbent == NULL || extraData(bestE)->fhat <= weight * bestF->f) {
      return getBestDHatAndSyncQueues();
    } else if(extraData(bestFhat)->fhat <= weight * bestF->f) {
      return getBestFHatAndSyncQueues();
    } else {
      return getBestFAndSyncQueues();
    }
  }

  void rebuildFocal() {
    double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;
    focal.clear();
    auto includeThis = [bound](Vertex* n) { return n != NULL && extraData(n)->fhat <= bound; };
    std::vector<Vertex*> range = open.getContiguousRange(includeThis);
    focal.createFromVector(range);
  }

  void addToQueues(Vertex *n) {
    double bound = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;
    open.insert(n);
    cleanup.push(n);
    if(extraData(n)->fhat <= bound) {
      focal.push(n);
    }
  }

  double getEdgeEffortBetweenCells(unsigned int c1, unsigned int c2) const {
    auto vertexAndEdges = edges.find(c1);
    assert(vertexAndEdges != edges.end());

    const auto &edges = vertexAndEdges->second;

    auto edge = edges.find(c2);
    assert(edge != edges.end());

    double effort = (extraData(vertices[c1])->clearance + extraData(vertices[c2])->clearance) * edge->second.weight * 0.5;

    return effort;
  }

  bool useEdgeEffort = false;

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

        double newValue = current->getVal() + (useEdgeEffort ? getEdgeEffortBetweenCells(current->getId(), kidIndex) : getEdgeCostBetweenCells(current->getId(), kidIndex));
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

  AbstractEffortEstimator effortEstimator;
  RBTree<Vertex, SortOnFHat> open;
  InPlaceBinaryHeap<Vertex, SortOnEHat> focal;
  InPlaceBinaryHeap<Vertex, SortOnF> cleanup;
  double oldBestFHatVal = 0;
  Vertex *incumbent = NULL;
  Vertex *lastVertex = NULL;
  double weight;
};

}

}