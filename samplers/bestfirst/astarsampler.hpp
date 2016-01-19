#pragma once

#include "../bestfirstsampler.hpp"

namespace ompl {

namespace base {

class AstarSampler : public ompl::base::BestFirstSampler {

  struct ExtraData {
    ExtraData() : heapIndex(std::numeric_limits<unsigned int>::max()) {}
    unsigned int heapIndex;

    static bool pred(const Vertex *a, const Vertex *b) {
      if(a->f == b->f) {
        return a->g > b->g;
      }
      return a->f < b->f;
    }
    static unsigned int getHeapIndex(const Vertex *r) {
      return ((ExtraData*)r->extraData)->heapIndex;
    }
    static void setHeapIndex(Vertex *r, unsigned int i) {
      ((ExtraData*)r->extraData)->heapIndex = i;
    }
  };

public:

  AstarSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    const FileMap &params) :
    BestFirstSampler(base, start, goal, params) {
      weight = params.exists("Weight") ? params.doubleVal("Weight") : 1.0;
    }

  virtual ~AstarSampler() {}

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

      std::vector<VertexWrapper*> wrappers;
      wrappers.reserve(vertices.size());
      std::vector<VertexHWrapper> hWrappers;
      hWrappers.reserve(vertices.size());
      for(auto v : vertices) {

        v->extraData = new ExtraData();


        hWrappers.emplace_back(v);
        wrappers.emplace_back(&hWrappers.back());
      }

      dijkstra(wrappers[1], wrappers);


      connected = !(std::isinf(vertices[0]->h) || std::isinf(vertices[1]->h));

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

    vertices[0]->f = vertices[0]->h * weight;
    vertices[0]->g = 0;
    auto neighbors = getNeighboringCells(0);
    for(auto n : neighbors) {
      vertices[n]->g = vertices[0]->g + getEdgeCostBetweenCells(0, n);
      vertices[n]->f = vertices[n]->g + vertices[n]->h * weight;
      open.push(vertices[n]);
    }
  }


  virtual bool sample(ompl::base::State *state) {
    if(randomNumbers.uniform01() < randomProbability) {
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

    target = open.peek();
    target->f *= peekPenalty;
    open.siftFromItem(target);

    ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
    vertexState = target->state;

    ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

    fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

    return true;
  }

  virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
    throw ompl::Exception("AstarSampler::sampleNear", "not implemented");
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

    double newG = fromVertex->g;

    if(isValidEdge(fromVertex->id, reachedVertex->id)) {
      newG += getEdgeCostBetweenCells(fromVertex->id, reachedVertex->id);
    } else {
      newG += FBiasedStateSampler::abstractDistanceFunction(fromVertex, reachedVertex);
    }
 
    if(newG < reachedVertex->g) {
      reachedVertex->g = newG;
      reachedVertex->f = newG + reachedVertex->h * weight;

      if(open.inHeap(reachedVertex)) {
        open.siftFromItem(reachedVertex);
      } else {
        open.push(reachedVertex);
      }
    } else {
      if(open.inHeap(reachedVertex)) {
        open.remove(reachedVertex);
      }
    }
    
    auto neighbors = getNeighboringCells(reachedVertex->id);
    for(auto n : neighbors) {
      double newChildG = newG + getEdgeCostBetweenCells(reachedVertex->id, n);

      if(newChildG < vertices[n]->g) {
        vertices[n]->g = newChildG;
        vertices[n]->f = vertices[n]->g + vertices[n]->h * weight;  
        
        if(open.inHeap(vertices[n])) {
          open.siftFromItem(vertices[n]);
        } else {
          open.push(vertices[n]);
        }
      }
    }
  }

protected:
  InPlaceBinaryHeap<Vertex, ExtraData> open;
  double weight;
  Vertex* target = NULL;
};

}

}