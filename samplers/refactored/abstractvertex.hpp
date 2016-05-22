#pragma once

namespace ompl {

namespace base {

namespace refactored {

class AbstractVertex {
public:
	AbstractVertex(unsigned int id) : id(id) {}
	
	virtual ~AbstractVertex() {}

	void addState(ompl::base::State *s) {
		assert(states.find(s) == states.end());
		states.insert(s);
	}

	void removeState(ompl::base::State *s) {
		auto iter = states.find(s);
		assert(states.size() > 0);
		assert(iter != states.end());
		states.erase(iter);
	}

	unsigned int id;

	double initG = std::numeric_limits<double>::infinity();
	double initH = std::numeric_limits<double>::infinity();

	std::unordered_set<ompl::base::State*> states;
	std::vector<ompl::base::State*> removedStates;
};

}

}

}