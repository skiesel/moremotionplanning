#pragma once

#include "../bestfirstsampler.hpp"

namespace ompl {

namespace base {

class DijkstraSampler : public ompl::base::BestFirstSampler {

  struct ExtraData {
    ExtraData() : heapIndex(std::numeric_limits<unsigned int>::max()) {}
    unsigned int heapIndex;

    static bool pred(const Vertex *a, const Vertex *b) {
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

  DijkstraSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    const FileMap &params) :
    BestFirstSampler(base, start, goal, params) {
    }

  virtual ~DijkstraSampler() {}

  virtual void initialize() {
    auto abstractStart = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getStartState(0);
    auto abstractGoal = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState();

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

    for(auto v : vertices) {
      v->extraData = new ExtraData();
    }

    vertices[0]->f = vertices[0]->g = 0;
    auto neighbors = getNeighboringCells(0);
    for(auto n : neighbors) {
      vertices[n]->f = vertices[n]->g = vertices[0]->g + FBiasedStateSampler::abstractDistanceFunction(vertices[0], vertices[n]);
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

    Vertex* target = open.peek();
    target->f *= peekPenalty;
    open.siftFromItem(target);

    ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
    vertexState = target->state;

    ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

    fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

    return true;
  }

  virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
    throw ompl::Exception("DijkstraSampler::sampleNear", "not implemented");
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

    double newG = fromVertex->g + FBiasedStateSampler::abstractDistanceFunction(fromVertex, reachedVertex);

    if(newG < reachedVertex->g) {
      reachedVertex->g = reachedVertex->f = newG;

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
        vertices[n]->g = vertices[n]->f = newChildG;
        
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
};

}

}