#pragma once

#include "dstarablevertexwrapper.hpp"

namespace ompl {

namespace base {

namespace refactored {

template <class Vertex, class Edge>
class DStarAble {

	typedef DStarAbleVertexWrapper Wrapper;
	typedef typename DStarAbleVertexWrapper::Key Key;

public:
	DStarAble(const std::vector<Vertex> &baseVertices, unsigned int goalID, Abstraction *abstraction, 
		std::function<Edge*(unsigned int, unsigned int)> getEdge,
		std::function<void(unsigned int, double)> callback) :
		goalID(goalID), abstraction(abstraction), getEdge(getEdge), callback(callback) {

		vertices.reserve(baseVertices.size());
		for(unsigned int i = 0; i < baseVertices.size(); i++) {
			vertices.emplace_back(i);
		}

		vertices[goalID].rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		U.push(&vertices[goalID]);
	}

	void updateVertex(unsigned int id) {
		Wrapper &s = vertices[id];
		if(s.id != goalID) {
			double minValue = std::numeric_limits<double>::infinity();
			auto neighbors = abstraction->getNeighboringCells(id);
			for(auto n : neighbors) {
				Edge *e = getEdge(id, n);
				double value = vertices[n].g + e->getEstimatedRequiredSamples();
				if(value < minValue) {
					minValue = value;
				}
			}
			s.rhs = minValue;
		}

		if(U.inHeap(&vertices[id])) {
			U.remove(&vertices[id]);
		}

		if(s.g != s.rhs) {
			s.key = calculateKey(id);
			U.push(&vertices[id]);
		}
	}

	void computeShortestPath() {
		while(!U.isEmpty()) {
			Wrapper &u = vertices[U.pop()->id];
			Key k_old = u.key;
			Key k_new = calculateKey(u.id);

			if(k_old < k_new) {
				u.key = k_new;
				U.push(&vertices[u.id]);
			}
			else if(u.g > u.rhs) {
				u.g = u.rhs;
				callback(u.id, u.g);

				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			} else {
				u.g = std::numeric_limits<double>::infinity();
				callback(u.id, u.g);

				updateVertex(u.id);
				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			}
		}
	}

	double getG(unsigned int i) const {
		return vertices[i].g;
	}

protected:
	Key calculateKey(unsigned int id) {
		Wrapper &s = vertices[id];
		Key key;
		key.first = std::min(s.g, s.rhs);
		key.second = std::min(s.g, s.rhs);
		return key;
	}

	std::vector<Wrapper> vertices;
	unsigned int goalID;
	Abstraction *abstraction = nullptr;
	std::function<Edge*(unsigned int, unsigned int)> getEdge;
	std::function<void(unsigned int, double)> callback;

	InPlaceBinaryHeap<Wrapper, Wrapper> U;
};

}

}

}