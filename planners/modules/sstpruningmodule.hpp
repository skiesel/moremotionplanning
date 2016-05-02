#pragma once

template <class MotionWithCost, class Motion>
class SSTPruningModuleBase {
public:
	SSTPruningModuleBase() {}
	virtual ~SSTPruningModuleBase() {}

	virtual void addStartState(MotionWithCost* m) = 0;
	virtual MotionWithCost* shouldPrune(MotionWithCost *m) = 0;
	virtual bool canSelectNode() const = 0;
	virtual MotionWithCost* selectNode(MotionWithCost *sample, const boost::shared_ptr<ompl::NearestNeighbors<Motion*>> &nn) const = 0;
	virtual void cleanupTree(MotionWithCost* m) = 0;
	virtual void clear() = 0;
};

template <class MotionWithCost, class Motion>
class NoSSTPruningModule : public SSTPruningModuleBase<MotionWithCost, Motion> {
public:
	NoSSTPruningModule() : SSTPruningModuleBase<MotionWithCost, Motion>() {}

	void addStartState(MotionWithCost* m) {}
	MotionWithCost* shouldPrune(MotionWithCost *m) { return nullptr; }
	bool canSelectNode() const { return false; }
	MotionWithCost* selectNode(MotionWithCost *sample, const boost::shared_ptr<ompl::NearestNeighbors<Motion*>> &nn) const { return nullptr; }
	void cleanupTree(MotionWithCost* m) {}
	void clear() {}
};

template <class MotionWithCost, class Motion>
class SSTPruningModule : public SSTPruningModuleBase<MotionWithCost, Motion> {
protected:
	class Witness : public MotionWithCost {
	public:

		Witness() {}

		Witness(const ompl::control::SpaceInformation *si) : MotionWithCost(si) {}

		void linkRep(MotionWithCost *lRep) {
			rep = lRep;
		}

		MotionWithCost *rep = nullptr;
	};

public:

	SSTPruningModule(const ompl::base::Planner *planner, const ompl::control::SpaceInformation *si, const ompl::base::OptimizationObjectivePtr& optimizationObjective, double selectionRadius, double pruningRadius) : SSTPruningModuleBase<MotionWithCost, Motion>(),
		si(si), optimizationObjective(optimizationObjective), selectionRadius(selectionRadius), pruningRadius(pruningRadius) {
		witnesses.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<MotionWithCost *>(planner));
		witnesses->setDistanceFunction(boost::bind(&SSTPruningModule::distanceFunction, this, _1, _2));
	}

	double distanceFunction(const MotionWithCost *a, const MotionWithCost *b) const {
		return si->distance(a->state, b->state);
	}

	void clear() {
		witnesses->clear();
	}

	void addStartState(MotionWithCost* m) {
		Witness *witness = new Witness(si);
		si->copyState(witness->state, m->state);
		witness->linkRep(m);
		witnesses->add(witness);
	}

	MotionWithCost* shouldPrune(MotionWithCost *m) {
		Witness *closestWitness = findClosestWitness(m);

		if(closestWitness->rep == m || optimizationObjective->isCostBetterThan(m->g, closestWitness->rep->g)) {

			MotionWithCost *oldRep = closestWitness->rep;
			closestWitness->linkRep(m);

			if(oldRep != m) {
				return oldRep;
			}

			return nullptr;
		}

		return m;
	}

	bool canSelectNode() const {
		return true;
	}

	MotionWithCost* selectNode(MotionWithCost *sample, const boost::shared_ptr<ompl::NearestNeighbors<Motion*>> &nn) const {
		std::vector<Motion*> ret;
		MotionWithCost *selected = nullptr;
		ompl::base::Cost bestCost = optimizationObjective->infiniteCost();
		nn->nearestR(sample, selectionRadius, ret);
		for(unsigned int i = 0; i < ret.size(); i++) {
			if(!((MotionWithCost*)ret[i])->inactive && optimizationObjective->isCostBetterThan(((MotionWithCost*)ret[i])->g, bestCost)) {
				bestCost = ((MotionWithCost*)ret[i])->g;
				selected = (MotionWithCost*)ret[i];
			}
		}
		if(selected == nullptr) {
			int k = 1;
			while(selected == nullptr) {
				nn->nearestK(sample, k, ret);
				for(unsigned int i=0; i < ret.size() && selected == nullptr; i++)
					if(!((MotionWithCost*)ret[i])->inactive)
						selected = (MotionWithCost*)ret[i];
				k += 5;
			}
		}
		return selected;
	}

	Witness* findClosestWitness(MotionWithCost *node) {
		if(witnesses->size() > 0) {
			Witness *closest = (Witness*)witnesses->nearest(node);
			if(distanceFunction(closest, node) > pruningRadius) {
				closest = new Witness(si);
				closest->linkRep(node);
				si->copyState(closest->state, node->state);
				witnesses->add(closest);
			}
			return closest;
		} else {
			Witness *closest = new Witness(si);
			closest->linkRep(node);
			si->copyState(closest->state, node->state);
			witnesses->add(closest);
			return closest;
		}
	}

	void cleanupTree(MotionWithCost *oldRep) {
		oldRep->inactive = true;
		while(oldRep != nullptr && oldRep->inactive && oldRep->numChildren == 0) {
			si->freeState(oldRep->state);
			si->freeControl(oldRep->control);
			MotionWithCost *oldRepParent = (MotionWithCost*)oldRep->parent;
			if(oldRepParent != nullptr && oldRepParent->numChildren > 0) {
				oldRepParent->numChildren--;
			}
			oldRep->parent = nullptr;
			delete oldRep;
			oldRep = oldRepParent;
		}
	}



protected:
	const ompl::control::SpaceInformation *si = NULL;
	const ompl::base::OptimizationObjectivePtr &optimizationObjective;
	double selectionRadius, pruningRadius;
	std::shared_ptr< ompl::NearestNeighbors<MotionWithCost*> > witnesses;
};

template <class MotionWithCost, class Motion>
class SSTStarPruningModule : public SSTPruningModule<MotionWithCost, Motion> {
public:
	SSTStarPruningModule(const ompl::base::Planner *planner, const ompl::control::SpaceInformation *si, const ompl::base::OptimizationObjectivePtr& optimizationObjective, double selectionRadius, double pruningRadius,
		double xi, double n0, double d, double l) : SSTPruningModule<MotionWithCost, Motion>(planner, si, optimizationObjective, selectionRadius, pruningRadius), xi(xi), n0(n0), d(d), l(l) {
		iterations = 0;
		iterationBound = n0;
		reductions = 0;
	}

	MotionWithCost* shouldPrune(const MotionWithCost *m) {
		iterations++;

		if(iterations >= iterationBound) {
			reductions++;
			this->selectionRadius *= xi;
			this->pruningRadius *= xi;
			iterationBound += (1 + log(reductions)) * pow(xi, -(d + l + 1) * reductions) * n0;
		}

		return SSTPruningModule<MotionWithCost, Motion>::shouldPrune(m);
	}
protected:
	double xi, n0, d, l;
	unsigned int iterations, iterationBound, reductions;

};