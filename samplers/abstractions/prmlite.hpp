#pragma once

#include "abstraction.hpp"

#include <ompl/base/SpaceInformation.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>


class PRMLite : public Abstraction {
public:
	PRMLite(const ompl::base::SpaceInformation *si, const ompl::base::State *start, const ompl::base::State *goal, const FileMap &params) :
		Abstraction(start, goal), prmSize(params.integerVal("PRMSize")), numEdges(params.integerVal("NumEdges")),
		stateRadius(params.doubleVal("StateRadius")) {

		resizeFactor = params.exists("PRMResizeFactor") ? params.doubleVal("PRMResizeFactor") : 2;

		//Stolen from tools::SelfConfig::getDefaultNearestNeighbors
		if(si->getStateSpace()->isMetricSpace()) {
			// if (specs.multithreaded)
			//  nn.reset(new NearestNeighborsGNAT<Vertex*>());
			//else
			nn.reset(new ompl::NearestNeighborsGNATNoThreadSafety<Vertex *>());
		} else {
			nn.reset(new ompl::NearestNeighborsSqrtApprox<Vertex *>());
		}

		nn->setDistanceFunction(boost::bind(&Abstraction::abstractDistanceFunction, this, _1, _2));
	}

	virtual void initialize(bool forceConnectedness = true) {
		generateVertices();
		generateEdges();

		while(forceConnectedness && !checkConnectivity()) {
			grow();
		}
	}

	virtual void grow() {
		for(unsigned int i = 0; i < prmSize; ++i) {
			vertices[i]->populatedNeighors = false;
			vertices[i]->neighbors.clear();
		}

		unsigned int oldPRMSize = prmSize;
		prmSize *= resizeFactor;
		vertices.resize(prmSize);
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();
		for(unsigned int i = oldPRMSize; i < prmSize; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}

		generateEdges();
	}

	virtual unsigned int getStartIndex() const {
		return 0;
	}

	virtual unsigned int getGoalIndex() const {
		return 1;
	}

	virtual bool supportsSampling() const {
		return false;
	}

	virtual ompl::base::State* sampleAbstractState(unsigned int index) {
		throw ompl::Exception("PRMLite::sampleAbstractState", "not supported");
		return NULL;
	}

	virtual unsigned int mapToAbstractRegion(const ompl::base::ScopedState<> &s) const {
		Vertex v(0);
		auto ss = globalParameters.globalAppBaseControl->getGeometricComponentState(s, -1); //-1 is intentional overflow on unsigned int
		v.state = ss.get();
		return nn->nearest(&v)->id;
	}

protected:
	void generateVertices() {
		Timer timer("Vertex Generation");
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();

		vertices.resize(prmSize);

		vertices[0] = new Vertex(0);
		vertices[0]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[0]->state, start);
		nn->add(vertices[0]);

		vertices[1] = new Vertex(1);
		vertices[1]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[1]->state, goal);
		nn->add(vertices[1]);

		for(unsigned int i = 2; i < prmSize; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}
	}

	void generateEdges() {
		Timer timer("Edge Generation");
		edges.clear();

		auto distanceFunc = nn->getDistanceFunction();

		for(Vertex *vertex : vertices) {
			edges[vertex->id];

			std::vector<Vertex *> neighbors;
			nn->nearestK(vertex, numEdges+1, neighbors);

			for(Vertex *neighbor : neighbors) {
				if(vertex->id == neighbor->id) continue;
				edges[vertex->id][neighbor->id] = Edge(neighbor->id);
				edges[neighbor->id][vertex->id] = Edge(vertex->id);
			}
		}
	}

	boost::shared_ptr< ompl::NearestNeighbors<Vertex *> > nn;
	unsigned int prmSize, numEdges;
	double stateRadius, resizeFactor;
};