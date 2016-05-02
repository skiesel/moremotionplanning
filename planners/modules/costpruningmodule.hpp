#pragma once

template <class MotionWithCost>
class CostPruningModule {
public:
	CostPruningModule(const ompl::base::OptimizationObjectivePtr &optimizationObjective) : optimizationObjective(optimizationObjective) {}
	
	virtual ~CostPruningModule() {}
	
	virtual bool shouldPrune(const MotionWithCost* m) const = 0;

protected:
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
};

template <class MotionWithCost>
class NoCostPruningModule : public CostPruningModule<MotionWithCost> {
public:
	NoCostPruningModule(const ompl::base::OptimizationObjectivePtr &optimizationObjective) : CostPruningModule<MotionWithCost>(optimizationObjective) {}

	virtual bool shouldPrune(const MotionWithCost* m) const {
		return false;
	}
};

template <class MotionWithCost>
class GCostPruningModule : public CostPruningModule<MotionWithCost> {
public:
	GCostPruningModule(const ompl::base::OptimizationObjectivePtr &optimizationObjective) : CostPruningModule<MotionWithCost>(optimizationObjective) {}

	virtual bool shouldPrune(const MotionWithCost* m) const {
		return !this->optimizationObjective->isSatisfied(m->g);
	}
};

template <class MotionWithCost>
class FCostPruningModule : public CostPruningModule<MotionWithCost> {
public:
	FCostPruningModule(const ompl::base::OptimizationObjectivePtr &optimizationObjective, const ompl::base::Goal *goal) : CostPruningModule<MotionWithCost>(optimizationObjective), goal(goal) {}

	virtual bool shouldPrune(const MotionWithCost* m) const {
		ompl::base::Cost h = this->optimizationObjective->costToGo(m->state, goal);
		return !this->optimizationObjective->isSatisfied(this->optimizationObjective->combineCosts(m->g, h));
	}

	const ompl::base::Goal *goal;
};
