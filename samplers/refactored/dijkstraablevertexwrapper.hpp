#pragma once

namespace ompl {

namespace base {

namespace refactored {

template <class Vertex>
class DijkstraAbleVertexWrapper {
public:
	static std::function<double(const Vertex*)> getVal;
	static std::function<void(Vertex*, double)> setVal;

	static bool pred(const DijkstraAbleVertexWrapper *a, const DijkstraAbleVertexWrapper *b) {
		return getVal(a->vert) < getVal(b->vert);
	}

	static unsigned int getHeapIndex(const DijkstraAbleVertexWrapper *r) {
		return r->heapIndex;
	}

	static void setHeapIndex(DijkstraAbleVertexWrapper *r, unsigned int i) {
		r->heapIndex = i;
	}

	unsigned int getId() const {
		return vert->id;
	}

	unsigned int heapIndex = std::numeric_limits<unsigned int>::max();
	Vertex *vert = nullptr;

	struct Parent {
		Parent(unsigned int parent, double cost) : parent(parent), cost(cost) {}
		static bool HeapCompare(const Parent *r1, const Parent *r2) {
			return r1->cost < r2->cost;
		}
		unsigned int parent;
		double cost;
	};

	bool addParent(unsigned int parent, double cost) {
		if(currentParent == NULL) {
			currentParent = new Parent(parent, cost);
			setVal(vert, currentParent->cost);
			return true;
		} else {
			if(cost < currentParent->cost) {
				parents.push_back(currentParent);
				std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
				currentParent = new Parent(parent, cost);
				setVal(vert, currentParent->cost);
				return true;
			} else {
				parents.push_back(new Parent(parent, cost));
				std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
				return false;
			}
		}
	}

	bool hasMoreParents() const {
		return !parents.empty() || currentParent != NULL;
	}

	unsigned int getBestParentIndex() const {
		assert(currentParent != NULL);
		return currentParent->parent;
	}

	void popBestParent() {
		assert(currentParent != NULL);

		delete currentParent;

		if(parents.size() == 0) {
			currentParent = NULL;
			setVal(vert, std::numeric_limits<double>::infinity());
		} else {
			currentParent = parents.front();
			std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
			parents.pop_back();
			setVal(vert, currentParent->cost);
		}
	}

	std::vector<Parent *> parents;
	Parent *currentParent;
};

template <class Vertex>
std::function<double(const Vertex*)> DijkstraAbleVertexWrapper<Vertex>::getVal;

template <class Vertex>
std::function<void(Vertex*, double)> DijkstraAbleVertexWrapper<Vertex>::setVal;

}

}

}