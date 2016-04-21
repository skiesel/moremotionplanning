#pragma once

#include "beastsampler_dstar.hpp"

namespace ompl {

namespace base {

class AnytimeBeastSampler : public ompl::base::BeastSampler_dstar {
protected:
	class DistributionTriple {
		class NormalDistribution {
		public:
			void addDataPoint(double value) {
				double oldMu = mu;
				mu += (value - oldMu) / (double)n;
				sigma = (n - 1) * sigma * sigma + (value - oldMu) * (value - mu);
				n++;
			}

			inline double getMu() const {
				return mu;
			}

			inline double getSigma() const {
				return sigma;
			}

			double getCDF(double value) const {
				return 0.5 * (1 + std::erf((value - mu) / (sqrt(2 * sigma))));
			}

			void add(const NormalDistribution &a, const NormalDistribution &b) {
				mu = a.mu + b.mu;
				sigma = a.sigma + b.sigma;
			}

		protected:
			double mu = 0, sigma = 0;
			unsigned int n = 1;
		};

	public:
		void addGValue(double value) {
			g.addDataPoint(value);
		}
		
		void addHValue(double value) {
			h.addDataPoint(value);
		}

		double getProbabilityFLessThanEqual(double incumbent) {
			f.add(g, h);
			//really it's like (incumbent - epsilon) but it's not going to matter
			return f.getCDF(incumbent);
		}

	protected:
		NormalDistribution g, h, f;
	};

public:
	AnytimeBeastSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
					BeastSampler_dstar(base, start, goal, gsr, params), optimizationObjective(optimizationObjective), goalPtr(goal) {}

	~AnytimeBeastSampler() {}

	virtual void initialize() {
		BeastSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

		costDistributions.resize(abstractionSize);

		ompl::base::ScopedState<> startStateSS(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
		ompl::base::ScopedState<> endStateSS(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());

		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
				getEdge(i, n);
				getEdge(n, i);
			}
		}

		vertices[goalID].rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		U.push(&vertices[goalID]);

		{
			Timer t("D* lite");
			computeShortestPath();
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				e.second->initialEffort = e.second->effort;
			}
		}

		vertices[startID].addState(startState);
		addOutgoingEdgesToOpen(startID);
	}

	virtual bool sample(ompl::base::State *from, ompl::base::State *to) {
		if(targetEdge != NULL) { //only will fail the first time through

			if(targetSuccess) {
				if(!addedGoalEdge && targetEdge->endID == goalID) {
					Edge *goalEdge = new Edge(goalID, goalID);
					goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
					goalEdge->effort = 1;
					open.push(goalEdge);
					addedGoalEdge = true;
				}

				if(targetEdge->interior) {
					updateSuccesfulInteriorEdgePropagation(targetEdge);
					updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
				} else {
					//edge has become interior
					targetEdge->interior = true;
					targetEdge->succesfulPropagation();
					updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
				}
			} else {
				targetEdge->failurePropagation();
				updateEdgeEffort(targetEdge, targetEdge->getEstimatedRequiredSamples() + vertices[targetEdge->endID].g);
			}

			updateVertex(targetEdge->startID);
			computeShortestPath();

			if(targetSuccess) {
				addOutgoingEdgesToOpen(targetEdge->endID);
			}
		}

		while(true) {
			assert(!open.isEmpty());

			targetEdge = open.peek();

			if(vertices[targetEdge->startID].states.size() <= 0 || !shouldExpand(targetEdge)) {
				open.pop();
			} else if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
				Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
																																		Abstraction::Edge::INVALID;
				targetEdge->updateEdgeStatusKnowledge(status);

				updateVertex(targetEdge->startID);
				computeShortestPath();
			} else {
				break;
			}
		}

		targetSuccess = false;

		if(targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
			goalSampler->sampleGoal(to);
		} else {
			si_->copyState(from, vertices[targetEdge->startID].sampleState());
			ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
			if(abstraction->supportsSampling()) {
				vertexState = abstraction->sampleAbstractState(targetEdge->endID);
			} else {
				vertexState = abstraction->getState(targetEdge->endID);
				ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
				fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
			}
		}
		return true;
	}

	void remove(ompl::base::State *state) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);
		vertices[newCellId].removeState(state);
	}

	void foundSolution(const ompl::base::Cost &incumbent) {
		incumbentCost = incumbent.value();
		targetEdge = NULL;
		addedGoalEdge = false;
		open.clear();

		for(auto &v : vertices) {
			v.clearStates();
		}

		vertices[startID].addState(startState);

		addOutgoingEdgesToOpen(startID);
	}

	void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = start;
		unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

		incomingState = end;
		unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

		costDistributions[endCellId].addGValue(endG);

		//pass this value back up through the tree branch?
		costDistributions[endCellId].addHValue(optimizationObjective->costToGo(end, goalPtr.get()).value());

		vertices[endCellId].addState(end);

		//if the planner chose the goal region first be careful not to dereference a null pointer
		if(targetEdge != NULL && endCellId == targetEdge->endID) {
			//this region will be added to open when sample is called again
			targetSuccess = true;
		} else  {
			addOutgoingEdgesToOpen(endCellId);
		}
	}

protected:

	bool shouldExpand(const Edge* e) {
		return true;//std::isinf(incumbentCost) || randomNumbers.uniform01() <= costDistributions[e->endID].getProbabilityFLessThanEqual(incumbentCost);
	}

	double incumbentCost = std::numeric_limits<double>::infinity();
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
	std::vector<DistributionTriple> costDistributions;
	const ompl::base::GoalPtr &goalPtr; 

};

}

}
