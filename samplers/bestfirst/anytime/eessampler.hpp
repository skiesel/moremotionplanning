#pragma once

#include "../../anytimebestfirstsampler.hpp"
#include "../../../structs/rbtree.hpp"

namespace ompl {

namespace base {

class AnytimeEESSampler : public ompl::base::AnytimeBestFirstSampler {

  struct ExtraData {
    ExtraData(double clearance) : clearance(clearance), fhat(std::numeric_limits<double>::infinity()),
    e(std::numeric_limits<double>::infinity()), ehat(std::numeric_limits<double>::infinity()),
    fIndex(std::numeric_limits<unsigned int>::max()), eHatIndex(std::numeric_limits<unsigned int>::max()) {}

    double clearance, fhat, e, ehat;
    unsigned int fIndex, eHatIndex;
  };

  static inline ExtraData* extraData(Vertex *v) { return ((ExtraData*)v->extraData); }
  static inline const ExtraData* extraData(const Vertex *v) { return ((ExtraData*)v->extraData); }

  struct VertexEWrapper : public VertexWrapper {
    VertexEWrapper(Vertex *vertex) : VertexWrapper(vertex) {}
    double getVal() const { return extraData(vertex)->e; }
    void setVal(double c) { extraData(vertex)->e = c; }
    bool sort(const VertexWrapper *n2) const { return extraData(this->vertex)->e < extraData(n2->vertex)->e; }
    bool operator<(const VertexWrapper *n2) const { return extraData(this->vertex)->e < extraData(n2->vertex)->e; }
  };

  struct SortOnF {
    static unsigned int getHeapIndex(const Vertex *n) { return extraData(n)->fIndex; }
    static void setHeapIndex(Vertex *n, unsigned int index) { extraData(n)->fIndex = index; }
    static bool pred(const Vertex *a, const Vertex *b) {
      if(a->f == b->f) return a->g > b->g;
      return a->f < b->f;
    }
  };

  struct SortOnEHat {
    static unsigned int getHeapIndex(const Vertex *n) { return extraData(n)->eHatIndex; }
    static void setHeapIndex(Vertex *n, unsigned int index) { extraData(n)->eHatIndex = index; }
    static bool pred(const Vertex *a, const Vertex *b) {
      if(extraData(a)->ehat == extraData(b)->ehat)
        return extraData(a)->fhat < extraData(b)->fhat;
      return extraData(a)->ehat < extraData(b)->ehat;
    }
  };

  struct SortOnFHat {
    static int compare(const Vertex *a, const Vertex *b) {
      if(extraData(a)->fhat == extraData(b)->fhat) return a->f - b->f;
      return extraData(a)->fhat - extraData(b)->fhat;
    }
  };

  class AbstractEffortEstimator : public ompl::base::StateCostIntegralObjective {
  public:
      AbstractEffortEstimator(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateCostIntegralObjective(si, false) {}
      ompl::base::Cost stateCost(const ompl::base::State* s) const { return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s)); }
  };

public:

  AnytimeEESSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    const FileMap &params) :
    AnytimeBestFirstSampler(base, start, goal, params),
    effortEstimator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()) {
      heuristicCorrection = 1.0;
      incumbentCost = std::numeric_limits<double>::infinity();
      weight = std::numeric_limits<double>::infinity();
    }

  virtual ~AnytimeEESSampler() {}

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

    extraData(vertices[1])->e = std::numeric_limits<double>::epsilon();

    vertices[0]->g = 0;
    vertices[0]->f = vertices[0]->h * weight;
    extraData(vertices[0])->fhat = vertices[0]->h * weight * heuristicCorrection;
    extraData(vertices[0])->ehat = extraData(vertices[0])->e;

    auto neighbors = getNeighboringCells(0);
    for(auto n : neighbors) {
      vertices[n]->g = vertices[0]->g + getEdgeCostBetweenCells(0, n);
      vertices[n]->f = vertices[n]->g + vertices[n]->h * weight;
      extraData(vertices[n])->fhat = vertices[n]->g + vertices[n]->h * weight * heuristicCorrection;
      extraData(vertices[n])->ehat = extraData(vertices[n])->e;

      addToQueues(vertices[n]);
    }
  }

  virtual bool sample(ompl::base::State *state) {
    if(randomNumbers.uniform01() <= randomProbability) {
      fullStateSampler->sampleUniform(state);
      return true;
    }

    Vertex *vertex = NULL;
    while(vertex == NULL) {
      vertex = selectVertex();
      //vertex = selectVertex_con();
      //vertex = selectVertex_opt();

      if(vertex == NULL) {
        break;
      } else if(vertex->f > incumbentCost) {
        vertex = NULL;
      }
    }

    if(vertex == NULL) {
      static bool warned = false;
      if(!warned)
        OMPL_WARN("Open is empty!");
      warned = true;
      fullStateSampler->sampleUniform(state);
      return true;
    }

    ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
    vertexState = vertex->state;

    ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

    fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

    return true;
  }

  virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
    throw ompl::Exception("AnytimeEESSampler::sampleNear", "not implemented");
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

    double oldBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;

    double newG = fromVertex->g;

    if(isValidEdge(fromVertex->id, reachedVertex->id)) {
      newG += getEdgeCostBetweenCells(fromVertex->id, reachedVertex->id);
    } else {
      newG += FBiasedStateSampler::abstractDistanceFunction(fromVertex, reachedVertex);
    }

    if(newG < reachedVertex->g) {
      reachedVertex->g = newG;
      reachedVertex->f = reachedVertex->g + reachedVertex->h * weight;
      extraData(reachedVertex)->fhat = reachedVertex->g + reachedVertex->h * weight * heuristicCorrection;
      extraData(reachedVertex)->ehat = extraData(reachedVertex)->e;

      addToQueues(reachedVertex);
    } else {
      removeFromQueues(reachedVertex);
    }

    auto neighbors = getNeighboringCells(reachedVertex->id);
    for(auto n : neighbors) {
      double newChildG = newG + getEdgeCostBetweenCells(reachedVertex->id, n);

      if(newChildG < vertices[n]->g && newChildG < incumbentCost) {
        vertices[n]->g = newChildG;
        vertices[n]->f = vertices[n]->g + vertices[n]->h * weight;
        extraData(vertices[n])->fhat = vertices[n]->g + vertices[n]->h * weight * heuristicCorrection;
        extraData(vertices[n])->ehat = extraData(vertices[n])->e;

        addToQueues(vertices[n]);
      }
    }

    double newBestFHatVal = open.isEmpty() ? std::numeric_limits<double>::infinity() : extraData(open.peekLeftmost())->fhat * weight;

    if(oldBestFHatVal != newBestFHatVal) {
      rebuildFocal();
    }
  }

  virtual void foundGoal(ompl::base::State*, ompl::base::State *toState, double solutionCost) {
    OMPL_INFORM("GOAL");


    ompl::base::ScopedState<> incomingState(si_->getStateSpace());
    incomingState = toState;

    Vertex v(0);
    auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
    v.state = ss.get();
    Vertex *reachedVertex = nn->nearest(&v);

    if(reachedVertex->g < incumbentCost) {
      double bestFValue = open.peekLeftmost()->f;
      incumbentCost = reachedVertex->g;
      weight = incumbentCost / bestFValue;
      heuristicCorrection = reachedVertex->g / solutionCost;
    }
  }

protected:
  virtual void generateVertices(const ompl::base::State *start, const ompl::base::State *goal, unsigned int howMany) {
    FBiasedStateSampler::generateVertices(start, goal, howMany);

    for(auto v : vertices) {
      v->extraData = new ExtraData(effortEstimator.stateCost(v->state).value());
    }
  }

  void removeFromQueues(Vertex *v) {
    if(cleanup.inHeap(v)) {
      cleanup.remove(v);
    }
    if(focal.inHeap(v)) {
      focal.remove(v);
    }
    open.remove(v);
  }

  Vertex* selectVertex() {
    Vertex* bestF = cleanup.isEmpty() ? NULL : cleanup.peek();
    Vertex* bestFhat = open.isEmpty() ? NULL : open.peekLeftmost();
    Vertex* bestEhat = focal.isEmpty() ? NULL : focal.peek();

    if(bestF == NULL || bestFhat == NULL || bestEhat == NULL) {
      return NULL;
    }

    if(extraData(bestEhat)->fhat <= weight * bestF->f) {
      extraData(bestEhat)->ehat *= peekPenalty;
      focal.siftFromItem(bestEhat);
      return bestEhat;
    } else if(extraData(bestFhat)->fhat <= weight * bestF->f) {
      return bestFhat;
    } else {
      return bestF;
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

    open.remove(n);
    open.insert(n);

    if(cleanup.inHeap(n)) {
      cleanup.siftFromItem(n);
    } else {
      cleanup.push(n);
    }
    if(extraData(n)->fhat <= bound) {
      if(focal.inHeap(n)) {
        focal.siftFromItem(n);
      } else {
        focal.push(n);
      }
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
  double incumbentCost, weight, heuristicCorrection;
};

}

}
