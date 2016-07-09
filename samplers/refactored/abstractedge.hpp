#pragma once

#include "../../structs/betadistribution.hpp"

namespace ompl {

namespace base {

namespace refactored {

class AbstractEdge {
public:
	static double validEdgeDistributionAlpha, validEdgeDistributionBeta, invalidEdgeDistributionAlpha, invalidEdgeDistributionBeta;

	struct AbstractEdgeComparator {
		bool operator()(const AbstractEdge *lhs, const AbstractEdge *rhs) const {
			if(fabs(lhs->effort - rhs->effort) <= 0.00001) {
				return lhs->startID < rhs->startID;
			}
			return lhs->effort < rhs->effort;
		}
	};

	AbstractEdge(unsigned int startID, unsigned int endID) : startID(startID), endID(endID), costReductionSuccess(1,0) {}

	virtual ~AbstractEdge() {}

	void updateEdgeStatusKnowledge(Abstraction::Edge::CollisionCheckingStatus newStatus) {
		status = newStatus;
		switch(status) {
			case Abstraction::Edge::VALID:
				alpha = validEdgeDistributionAlpha;
				beta = validEdgeDistributionBeta;
				break;
			case Abstraction::Edge::INVALID:
				alpha = invalidEdgeDistributionAlpha;
				beta = invalidEdgeDistributionBeta;
				break;

			case Abstraction::Edge::UNKNOWN:
				// alpha = unknownEdgeDistributionAlpha;
				// beta = unknownEdgeDistributionBeta;
				break;
		}
	}

	double getProbabilityOfCostReduction() const {
		return costReductionSuccess.getProbabilityOfSuccess();
	}

	void costReduced() {
		costReductionSuccess.incrementAlpha();
	}

	void costNotReduced() {
		costReductionSuccess.incrementBeta();
	}

	void succesfulPropagation() {
		alpha++;
	}

	void failurePropagation() {
		beta++;
	}

	double getEstimatedRequiredSamples() const {
		double probability = alpha / (alpha + beta);
		double estimate = 1. / probability;
		return estimate;
	}

	double getHypotheticalEstimatedSamplesAfterPositivePropagation(unsigned int numberOfStates) const {
		double additive = (1. / (double)numberOfStates);
		double probability = (alpha + additive) / (alpha + additive + beta);
		double estimate = 1. / probability;
		return estimate;
	}

	void rewardHypotheticalSamplesAfterPositivePropagation(unsigned int numberOfStates) {
		double additive = (1. / ((double)numberOfStates - 1));
		alpha += additive;
	}

	unsigned int startID, endID, interiorToNextEdgeID;
		
	Abstraction::Edge::CollisionCheckingStatus status = Abstraction::Edge::UNKNOWN;
	
	double alpha = validEdgeDistributionAlpha;
	double beta = validEdgeDistributionBeta;

	double effort = std::numeric_limits<double>::infinity();
	
	bool interior = false;
	BetaDistribution costReductionSuccess;
};

	double AbstractEdge::validEdgeDistributionAlpha = 0;
	double AbstractEdge::validEdgeDistributionBeta = 0;

	double AbstractEdge::invalidEdgeDistributionAlpha = 0;
	double AbstractEdge::invalidEdgeDistributionBeta = 0;

}

}

}