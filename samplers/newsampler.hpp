#pragma once

#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <thread>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/goals/GoalState.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLStateValidityChecker.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class NewSampler : public ompl::base::FBiasedStateSampler {

  struct ExtraData {
    ExtraData(double clearance) : clearance(clearance) {}
    double clearance;
  };

  class AbstractEffortEstimator : public ompl::base::StateCostIntegralObjective {
  public:
      AbstractEffortEstimator(const ompl::base::SpaceInformationPtr& si) :
          ompl::base::StateCostIntegralObjective(si, false) {}

      ompl::base::Cost stateCost(const ompl::base::State* s) const {
          return ompl::base::Cost(1 / si_->getStateValidityChecker()->clearance(s));
      }
  };

  double abstractDistanceFunction(const Vertex *a, const Vertex *b) const {
    assert(a->state != NULL);
    assert(b->state != NULL);
    return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(a->state, b->state);
  }

public:

  NewSampler( ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    unsigned int prmSize, unsigned int numEdges, double omega, double stateRadius) :
    FBiasedStateSampler(base, start, goal, prmSize, numEdges, omega, stateRadius),
    motionValidator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->getMotionValidator()),
    effortEstimator(globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()) {

    }

  virtual ~NewSampler() {}

  virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
    throw ompl::Exception("NewSampler::sampleNear", "not implemented");
    return false;
  }

  void reached(ompl::base::State *state) {}

protected:
  virtual void generateVertices(const ompl::base::State *start, const ompl::base::State *goal, unsigned int howMany) {
    FBiasedStateSampler::generateVertices(start, goal, howMany);
    for(auto v : vertices) {
      v->extraData = new ExtraData(effortEstimator.stateCost(v->state).value());  
    }
  }

  inline double getClearance(const Vertex *v) const {
    return ((ExtraData*)v->extraData)->clearance;
  }

  virtual void generateEdges(unsigned int howManyConnections) {
    Timer timer("Edge Generation");
    auto distanceFunc = nn->getDistanceFunction();

    for(Vertex* vertex : vertices) {
      edges[vertex->id];

      std::vector<Vertex *> neighbors;
      nn->nearestK(vertex, howManyConnections+1, neighbors);
      // nn->nearestR(vertex, 1., neighbors);

      for(Vertex* neighbor : neighbors) {
        double distance = distanceFunc(vertex, neighbor);
        if(distance == 0) continue;

        double effort = 0.5 * (getClearance(vertex) + getClearance(neighbor)) / distance;

        edges[vertex->id][neighbor->id] = Edge(neighbor->id, effort);
        edges[neighbor->id][vertex->id] = Edge(vertex->id, effort);
      }
    }
  }

  ompl::base::MotionValidatorPtr motionValidator;
  AbstractEffortEstimator effortEstimator;
};

}

}
