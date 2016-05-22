#pragma once

#include "dijkstraablevertexwrapper.hpp"
#include "../abstractions/abstraction.hpp"

namespace ompl {

namespace base {

namespace refactored {

template <class Vertex>
class DijkstraAble {
public:

	typedef DijkstraAbleVertexWrapper<Vertex> Vert;

	DijkstraAble() {}

	virtual ~DijkstraAble() {}

	void dijkstra(unsigned int start, std::vector<Vertex> &vertices, Abstraction *abstraction,
		std::function<double(const Vertex*)> getVal,
		std::function<void(Vertex*, double)> setVal) {

		Vert::getVal = getVal;
		Vert::setVal = setVal;

		std::vector<Vert> wrappers(vertices.size());
		for(unsigned int i = 0; i < wrappers.size(); i++) {
			wrappers[i].vert = &vertices[i];
		}

		InPlaceBinaryHeap<Vert, Vert> open;
		std::unordered_set<unsigned int> closed;
		setVal(wrappers[start].vert, 0);
		open.push(&wrappers[start]);
		closed.insert(wrappers[start].getId());

		while(!open.isEmpty()) {
			Vert *current = open.pop();

			if(current->getId() != wrappers[start].getId()) {
				unsigned int parentIndex = current->getBestParentIndex();
				if(!abstraction->isValidEdge(parentIndex, current->getId())) {
					//this will update the value of the vertex if needed
					current->popBestParent();
					if(current->hasMoreParents()) {
						open.push(current);
					}
					continue;
				}
			}

			closed.insert(current->getId());

			if(closed.size() == wrappers.size()) break;

			std::vector<unsigned int> kids = abstraction->getNeighboringCells(current->getId());
			double curBaseVal = getVal(current->vert);
			for(unsigned int kidIndex : kids) {
				if(closed.find(kidIndex) != closed.end()) continue;

				double newValue = curBaseVal + abstraction->abstractDistanceFunctionByIndex(current->getId(), kidIndex);
				Vert *kid = &wrappers[kidIndex];

				//this will update the value of the vertex if needed
				bool addedBetterParent = kid->addParent(current->getId(), newValue);

				if(open.inHeap(kid)) {
					if(addedBetterParent) {
						open.siftFromItem(kid);
					}
				} else {
					open.push(kid);
				}
			}
		}
	}

protected:

};

}

}

}