#pragma once

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
	enum LABELS {
		G = 0,
		H = 1,
		F = 2,
		SCORE = 3,
		TOTAL = 4,
	};

protected:
	struct Vertex : public FBiasedStateSampler::Vertex {
		Vertex() {}
		Vertex(unsigned int id, unsigned int numVals = 0) : FBiasedStateSampler::Vertex(id, numVals),
			pdfID(std::numeric_limits<unsigned int>::max()), inExteriorPDF(false), touched(false) {}
		unsigned int pdfID;
		bool inExteriorPDF, touched;
	};

public:
	FBiasedShellStateSampler(ompl::base::SpaceInformation *base, ompl::base::State *start_, const ompl::base::GoalPtr &goal,
	                         const FileMap &params) :
		FBiasedStateSampler(base, start_, goal, params), shellPreference(params.doubleVal("ShellPreference")),
		shellDepth(params.doubleVal("ShellDepth")), start(base->getStateSpace()->allocState()) {
		base->getStateSpace()->copyState(start, start_);
	}

	virtual ~FBiasedShellStateSampler() {}

	virtual void initialize() {		
		abstraction->initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.clear();
		vertices.reserve(abstractionSize);
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i, TOTAL);
		}

		std::vector<VertexWrapper *> wrappers;
		wrappers.reserve(vertices.size());
		std::vector<VertexGWrapper> gWrappers;
		gWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			gWrappers.emplace_back(&vertices[i]);
			wrappers.emplace_back(&gWrappers.back());
		}

		dijkstra(wrappers[abstraction->getStartIndex()], wrappers);

		//the connectivity check being done on abstraction initialization should assure this
		assert(!std::isinf(vertices[abstraction->getGoalIndex()].vals[G]));

		std::vector<VertexHWrapper> hWrappers;
		hWrappers.reserve(vertices.size());
		for(unsigned int i = 0; i < vertices.size(); ++i) {
			hWrappers.emplace_back(&vertices[i]);
			wrappers[i] = &hWrappers.back();
		}

		dijkstra(wrappers[abstraction->getGoalIndex()], wrappers);

#ifdef STREAM_GRAPHICS
		generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].vals[SCORE]; }, "fbiasedshell.prm");
#endif

		reached(start);
	}

	virtual bool sample(ompl::base::State *state) {
		Vertex *randomVertex = NULL;
		if(randomNumbers.uniform01() < shellPreference && !exterior.isEmpty()) {
			randomVertex = exterior.sample();
		} else if(!interior.isEmpty()) {
			randomVertex = interior.sample();
		}

		if(randomVertex == NULL) {
			return UniformValidStateSampler::sample(state);
		}

		ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		if(abstraction->supportsSampling()) {
			vertexState = abstraction->sampleAbstractState(randomVertex->id);
		}
		else {
			vertexState = abstraction->getState(randomVertex->id);
		}

		ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);

		fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

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
		unsigned int start = abstraction->mapToAbstractRegion(incomingState);

		if(vertices[start].touched) {
			return;
		}

		InPlaceBinaryHeap<node, node> open;
		std::unordered_set<unsigned int> closed;
		std::unordered_map<unsigned int, node *> lookup;


		lookup[start] = new node(start, 0, 0);

		if(vertices[start].inExteriorPDF) {
			auto el = exterior.remove(vertices[start].pdfID);
			if(el != NULL) {
				el->getData()->pdfID = el->getId();
			}
		}

		auto el = interior.add(&vertices[start], vertices[start].vals[SCORE]);
		vertices[start].pdfID = el->getId();
		vertices[start].inExteriorPDF = false;
		vertices[start].touched = true;

		open.push(lookup[start]);
		while(!open.isEmpty()) {
			node *current = open.pop();
			closed.insert(current->id);

			std::vector<unsigned int> kids = getNeighboringCells(current->id);
			for(unsigned int kid : kids) {
				if(closed.find(kid) != closed.end() || vertices[kid].touched) continue;

				double newValue = current->value + abstraction->abstractDistanceFunctionByIndex(current->id, kid);

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
			if(!(vertices[n].inExteriorPDF || vertices[n].touched)) {
				auto el = exterior.add(&vertices[n], vertices[n].vals[SCORE]);
				vertices[n].pdfID = el->getId();
				vertices[n].inExteriorPDF = true;
			}
		}
	}

	std::vector<Vertex> vertices;
	ProbabilityDensityFunction<Vertex> interior, exterior;
	ompl::base::State *start;
	double shellPreference, shellDepth;
	ompl::RNG randomNumbers;
};

}

}
