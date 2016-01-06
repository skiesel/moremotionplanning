#include <unordered_set>
#include <unordered_map>

#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLContinuousMotionValidator.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {

namespace base {

class FBiasedShellStateSampler : public ompl::base::FBiasedStateSampler {

	struct ExtraData {
		ExtraData() : inExteriorPDF(false), touched(false) {}
		bool inExteriorPDF, touched;
	};

public:
	FBiasedShellStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
    double omega, double stateRadius, double shellPreference, double shellDepth) :
		FBiasedStateSampler(base, start, goal, omega, stateRadius, false), shellPreference(shellPreference), shellDepth(shellDepth) {

    for(auto vertex : vertices) {
      vertex->extraData = new ExtraData();
    }

// dumpToStderr();

    reached(start);
	}

	virtual ~FBiasedShellStateSampler() {
    for(auto vertex : vertices) {
      delete (ExtraData*)vertex->extraData;
    }
  }

	virtual bool sample(ompl::base::State *state) {
    Vertex *randomVertex = NULL;
    if(randomNumbers.uniform01() < shellPreference && !exterior.isEmpty()) {
      randomVertex = exterior.sample();
    } else if(!exterior.isEmpty()) {
      randomVertex = interior.sample();
    }
    
    if(randomVertex == NULL) {
      return UniformValidStateSampler::sample(state);
    }

    ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
    vertexState = randomVertex->state;

    ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

    fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);
    // fullStateSampler->sampleUniform(state);

    return true;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("FBiasedShellStateSampler::sampleNear", "not implemented");
		return false;
	}

  void reached(ompl::base::State *state) {
    reached(state, shellDepth);
  }

protected:

  inline bool wasTouched(Vertex* v) const {
    return ((ExtraData*)v->extraData)->touched;
  }

  inline void setTouched(Vertex* v) {
    ((ExtraData*)v->extraData)->touched = true;
  }

  inline bool isInExteriorPDF(Vertex *v) const {
    return ((ExtraData*)v->extraData)->inExteriorPDF;
  }

  inline void setInExteriorPDF(Vertex *v, bool inExteriorPDF) {
    ((ExtraData*)v->extraData)->inExteriorPDF = inExteriorPDF;
  }

  void reached(ompl::base::State *state, double shellDepth) {
    //we have a local shellDepth passed in, and a object level shellDepth -- maybe we want to alter it during runtime?
    struct node {
      node(unsigned int id, unsigned int depth, double value) : id(id), depth(depth), value(value) {}

      static bool pred(const node *a, const node *b) {
        return a->value < b->value;
      }
      static unsigned int getHeapIndex(const node *r) {
        return r->heapIndex;
      }
      static void setHeapIndex(node *r, unsigned int i) {
        r->heapIndex = i;
      }

      unsigned int id, depth, heapIndex;
      double value;
    };


    ompl::base::ScopedState<> incomingState(si_->getStateSpace());
    incomingState = state;

    Vertex v(0);
    auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(incomingState, 0);
    v.state = ss.get();
    unsigned int start = nn->nearest(&v)->id;

    if(wasTouched(vertices[start])) {
      return;
    }

    InPlaceBinaryHeap<node, node> open;
    std::unordered_set<unsigned int> closed;
    std::unordered_map<unsigned int, node *> lookup;


    lookup[start] = new node(start, 0, 0);

    if(isInExteriorPDF(vertices[start])) {
      auto el = exterior.remove(vertices[start]->pdfID);
      if(el != NULL) {
        el->getData()->pdfID = el->getId();
      }
    }

    auto el = interior.add(vertices[start], vertices[start]->score);
    vertices[start]->pdfID = el->getId();
    setInExteriorPDF(vertices[start], false);
    setTouched(vertices[start]);

    open.push(lookup[start]);
    while(!open.isEmpty()) {
      node *current = open.pop();
      closed.insert(current->id);

      std::vector<unsigned int> kids = getNeighboringCells(current->id);
      for(unsigned int kid : kids) {
        if(closed.find(kid) != closed.end()) continue;

        double newValue = current->value + getEdgeCostBetweenCells(current->id, kid);

        if(newValue <= shellDepth /*|| current->depth == 0*/) {
          if(lookup.find(kid) != lookup.end()) {
            if(newValue < lookup[kid]->value) {
              lookup[kid]->value = newValue;
              lookup[kid]->depth = current->depth + 1;
              open.siftFromItem(lookup[kid]);
            }
          } else {
            lookup[kid] = new node(kid, current->depth + 1, newValue);
            open.push(lookup[kid]);
          }
        }
      }
    }

    for(auto entry : lookup) {
      auto n = entry.first;
      delete entry.second;
      if(!(isInExteriorPDF(vertices[n]) || wasTouched(vertices[n]))) {
        auto el = exterior.add(vertices[n], vertices[n]->score);
        vertices[n]->pdfID = el->getId();
        setInExteriorPDF(vertices[n], true);
      }
    }
  }

	ProbabilityDensityFunction<Vertex> interior, exterior;
	double shellPreference, shellDepth;
	ompl::RNG randomNumbers;
};

}

}
