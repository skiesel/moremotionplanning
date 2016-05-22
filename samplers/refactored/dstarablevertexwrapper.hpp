#pragma once

namespace ompl {

namespace base {

namespace refactored {

class DStarAbleVertexWrapper {
public:
	struct Key {
		bool operator<(const Key& k) const {
			if(first == k.first)
				return second < k.second;
			return first < k.first;
		}
		double first, second;
	};

	DStarAbleVertexWrapper(unsigned int id) : id(id) {}

	static bool pred(const DStarAbleVertexWrapper *a, const DStarAbleVertexWrapper *b) {
		return a->key < b->key;
	}
	static unsigned int getHeapIndex(const DStarAbleVertexWrapper *r) {
		return r->heapIndex;
	}
	static void setHeapIndex(DStarAbleVertexWrapper *r, unsigned int i) {
		r->heapIndex = i;
	}

	unsigned int id;
	unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

	Key key;
	double g = std::numeric_limits<double>::infinity();
	double rhs = std::numeric_limits<double>::infinity();

};

}

}

}