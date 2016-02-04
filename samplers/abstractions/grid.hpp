#pragma once

#include "abstraction.hpp"

class Grid : public Abstraction {
	struct Vertex : public Abstraction::Vertex {
		Vertex(unsigned int id) : Abstraction::Vertex(id) {}

		bool hasGridCenter = false;
		std::vector<double> gridCenter;
		bool hasGridCoordinate = false;
		std::vector<unsigned int> gridCoordinate;
	};

public:
	Grid(const ompl::base::State *start, const ompl::base::State *goal, const FileMap &params) : Abstraction(start, goal) {
		hintSize = params.doubleVal("GridHintSize");
		useDiagonalNeighbors = params.exists("GridConnectivity") && params.stringVal("GridConnectivity").compare("Diagonals") == 0;
		resizeFactor = params.exists("GridResizeFactor") ? params.doubleVal("GridResizeFactor") : 0.5;
	}

	virtual void initialize(bool forceConnectedness = true) {
		unsigned int stateVars = globalParameters.abstractBounds.low.size();

		sizes = std::vector<double>(stateVars, hintSize);
		nextResize = 0;

		dimensionRanges.resize(stateVars);
		for(unsigned int i = 0; i < stateVars; ++i) {
			dimensionRanges[i] = globalParameters.abstractBounds.high[i] - globalParameters.abstractBounds.low[i];
		}

		discretizationSizes.resize(stateVars);
		discreteDimensionSizes.resize(stateVars);

		generateOffsets();
		
		cellCount = 0;

		reinitialize();

		while(forceConnectedness && !checkConnectivity()) {
			grow();
		}
	}

	virtual unsigned int getStartIndex() const {
		return startIndex;
	}

	virtual unsigned int getGoalIndex() const {
		return goalIndex;
	}

	virtual bool supportsSampling() const {
		return true;
	}

	virtual ompl::base::State* sampleAbstractState(unsigned int index) {
		static bool warned = false;
		if(!warned) {
			OMPL_WARN("Grid::sampleAbstractState... this is not actually a truly uniform sampling strategy!");
			warned = true;
		}

		auto point = getGridCenter(index);

		std::vector<double> sample(discretizationSizes.size());
		for(unsigned int i = 0; i < discretizationSizes.size(); ++i) {
			sample[i] = point[i] - (discretizationSizes[i] / 2) + (discretizationSizes[i] * randomNumbers.uniform01());
		}

		ompl::base::State *state = globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->allocState();
		globalParameters.copyVectorToAbstractState(state, point);
		return state;
	}

	virtual unsigned int mapToAbstractRegion(const ompl::base::ScopedState<> &s) const {
		return mapToAbstractRegion(s.get());
	}

	unsigned int mapToAbstractRegion(const ompl::base::State *s) const {
		std::vector<double> point;
		globalParameters.copyAbstractStateToVector(point, s);
		return getIndex(point);
	}

	virtual void grow() {
		sizes[nextResize] *= resizeFactor;
		if(++nextResize >= sizes.size()) nextResize = 0;
		reinitialize();
	}

protected:
	void generateVertices(unsigned int oldCount, unsigned int newCount) {
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();

		cellCount = newCount;

		vertices.reserve(cellCount);

		for(unsigned int i = 0; i < oldCount; ++i) {
			vertices[i]->populatedNeighors = false;
			vertices[i]->neighbors.clear();
			((Grid::Vertex*)vertices[i])->hasGridCenter = false;
			((Grid::Vertex*)vertices[i])->hasGridCoordinate = false;

			auto gridPoint = getGridCoordinates(i);
			auto point = getGridCenter(i);

			globalParameters.copyVectorToAbstractState(vertices[i]->state, point);
		}

		for(unsigned int i = oldCount; i < cellCount; ++i) {
			vertices.emplace_back(new Vertex(i));
			vertices.back()->state = abstractSpace->allocState();

			auto gridPoint = getGridCoordinates(i);
			auto point = getGridCenter(i);

			globalParameters.copyVectorToAbstractState(vertices.back()->state, point);
		}
	}

	virtual void generateEdges() {
		edges.clear();

		for(unsigned int i = 0; i < vertices.size(); ++i) {
			Vertex *vertex = (Grid::Vertex*)vertices[i];
			edges[vertex->id];

			auto discreteCoordinate = getGridCoordinates(i);

			std::vector<unsigned int> neighbors = getNeighbors(discreteCoordinate);

			for(auto n : neighbors) {
				Vertex *neighbor = (Grid::Vertex*)vertices[n];
				if(vertex->id == neighbor->id) continue;
				
				edges[vertex->id][neighbor->id] = Edge(neighbor->id);
				edges[neighbor->id][vertex->id] = Edge(vertex->id);
			}
		}
	}

	void reinitialize() {
		for(unsigned int i = 0; i < discretizationSizes.size(); ++i) {
			discretizationSizes[i] = sizes[i] * dimensionRanges[i];
		}

		unsigned int newCellCount = 1;

		for(unsigned int i = 0; i < discreteDimensionSizes.size(); i++) {

			if(dimensionRanges[i] == 0) {
				discreteDimensionSizes[i] = 1;
			} else {
				discreteDimensionSizes[i] = ceil(dimensionRanges[i] / discretizationSizes[i]);
			}
			newCellCount *= discreteDimensionSizes[i];
		}

		generateVertices(cellCount, newCellCount);
		generateEdges();

		startIndex = mapToAbstractRegion(start);
		goalIndex = mapToAbstractRegion(goal);
	}

	void generateOffsets() {
		gridNeighbors.clear();
		std::vector<int> gridCoordinate(globalParameters.abstractBounds.low.size(), 0);
		if(useDiagonalNeighbors) {
			populateNeighborsWithDiagonals(gridCoordinate, 0);
		} else {
			populateNeighborsGrid();	
		}	
	}

	std::vector<unsigned int> getNeighbors(const std::vector<unsigned int> &discreteCoordinate) const {
		std::vector<unsigned int> neighbors;

		for(auto neighbor : gridNeighbors) {
			std::vector<unsigned int> coord(discreteCoordinate);
			bool valid = true;
			for(unsigned int i = 0; i < neighbor.size(); ++i) {
				if((coord[i] == 0 && neighbor[i] < 0) ||
					(coord[i] + neighbor[i] >= discreteDimensionSizes[i])) { valid = false; break; }
				coord[i] += neighbor[i];
			}
			if(valid) {
				neighbors.push_back(getIndex(coord));
			}
		}

		return neighbors;
	}

	void populateNeighborsGrid() {
		gridNeighbors.clear();

		std::vector<int> gridCoordinate(globalParameters.abstractBounds.low.size(), 0);

		for(unsigned int i = 0; i < gridCoordinate.size(); ++i) {
			gridCoordinate[i] += 1;
			gridNeighbors.push_back(gridCoordinate);
			
			gridCoordinate[i] -= 2;

			gridNeighbors.push_back(gridCoordinate);
			gridCoordinate[i] += 1;
		}
	}

	void populateNeighborsWithDiagonals(std::vector<int> &gridCoordinate, unsigned int index) {
		if(index >= gridCoordinate.size()) return;

		populateNeighborsWithDiagonals(gridCoordinate, index+1);

		gridCoordinate[index] += 1;
		gridNeighbors.push_back(gridCoordinate);
		populateNeighborsWithDiagonals(gridCoordinate, index+1);
		
		gridCoordinate[index] -= 2;

		gridNeighbors.push_back(gridCoordinate);
		populateNeighborsWithDiagonals(gridCoordinate, index+1);

		gridCoordinate[index] += 1;
	}

	std::vector<double> getGridCenter(unsigned int n) const {
		if(((Grid::Vertex*)vertices[n])->hasGridCenter) {
			return ((Grid::Vertex*)vertices[n])->gridCenter;
		}

		std::vector<double> point;

		point.push_back(globalParameters.abstractBounds.low[0] + (double)(n % discreteDimensionSizes[0]) * discretizationSizes[0] + discretizationSizes[0]  * 0.5);
		if(discreteDimensionSizes.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < discreteDimensionSizes.size(); i++) {
				previousDimSizes *= discreteDimensionSizes[i];
				point.push_back(globalParameters.abstractBounds.low[i] + (double)(n / previousDimSizes % discreteDimensionSizes[i]) * discretizationSizes[i] + discretizationSizes[1]  * 0.5);
			}
		}

		((Grid::Vertex*)vertices[n])->hasGridCenter = true;
		((Grid::Vertex*)vertices[n])->gridCenter = point;

		return point;
	}

	std::vector<unsigned int> getGridCoordinates(unsigned int n) const {
		if(((Grid::Vertex*)vertices[n])->hasGridCoordinate) {
			return ((Grid::Vertex*)vertices[n])->gridCoordinate;
		}

		std::vector<unsigned int> coordinate;

		coordinate.push_back(n % discreteDimensionSizes[0]);
		if(discreteDimensionSizes.size() > 1) {
			unsigned int previousDimSizes = 1;
			for(unsigned int i = 1; i < discreteDimensionSizes.size(); i++) {
				previousDimSizes *= discreteDimensionSizes[i];
				coordinate.push_back(n / previousDimSizes % discreteDimensionSizes[i]);
			}
		}

		((Grid::Vertex*)vertices[n])->hasGridCoordinate = true;
		((Grid::Vertex*)vertices[n])->gridCoordinate = coordinate;

		return coordinate;
	}

	unsigned int getIndex(const std::vector<double> &point) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < discretizationSizes.size(); i++) {

			unsigned int which = floor((point[i] - globalParameters.abstractBounds.low[i]) / discretizationSizes[i]);

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= discreteDimensionSizes[j];
			}

			index += which * offset;
		}
		return index;
	}

	unsigned int getIndex(const std::vector<unsigned int> &gridCoordinate) const {
		unsigned int index = 0;

		for(unsigned int i = 0; i < discreteDimensionSizes.size(); i++) {

			double offset = 1;
			for(unsigned int j = 0; j < i; j++) {
				offset *= discreteDimensionSizes[j];
			}

			index += gridCoordinate[i] * offset;
		}
		return index;
	}

	double hintSize, resizeFactor;
	std::vector<double> sizes;
	unsigned int nextResize, cellCount, startIndex, goalIndex;
	
	std::vector< std::vector<int> > gridNeighbors;

	std::vector<unsigned int> discreteDimensionSizes;
	std::vector<double> discretizationSizes, dimensionRanges;

	bool useDiagonalNeighbors;
	ompl::RNG randomNumbers;
};