#pragma once

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/GenericParam.h>

#include "abstractions/abstraction.hpp"
#include "abstractions/prmlite.hpp"
#include "abstractions/grid.hpp"

namespace ompl {

namespace base {

class AbstractionBasedSampler : public ompl::base::UniformValidStateSampler {
public:
	AbstractionBasedSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
		const FileMap &params) : UniformValidStateSampler(base), fullStateSampler(base->allocStateSampler()),
		abstraction(NULL), stateRadius(params.doubleVal("StateRadius")) {

		auto abstractStart = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getStartState(0);
		auto abstractGoal = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState();

		std::string abstractionType = params.exists("AbstractionType") ? params.stringVal("AbstractionType") : "PRM";
		if(abstractionType.compare("PRM") == 0) {
			abstraction = new PRMLite(base, abstractStart, abstractGoal, params);
		} else if(abstractionType.compare("GRID") == 0) {
			abstraction = new ::Grid(abstractStart, abstractGoal, params);
		}
		else {
			throw ompl::Exception("AbstractionBasedSampler", "unrecognized abstraction type");
		}
	}

	virtual ~AbstractionBasedSampler() {
		delete abstraction;
	}

	virtual void initialize() = 0;
	virtual bool sample(ompl::base::State *state) = 0;
	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) = 0;

	std::vector<unsigned int> getNeighboringCells(unsigned int index) const {
		return abstraction->getNeighboringCells(index);
	}

protected:
	std::vector<double> getColor(double min, double max, double value) const {
		std::vector<double> color(3);

		value = ((value - min) / (max - min)) * 765;

		if(value < 255) {
			color[0] = 0;
			color[1] = value / 2;
			color[2] = 255 - value;
		} else if(value < 510) {
			double relVal = value - 255;
			color[0] = relVal;
			color[1] = (relVal + 255) / 2;
			color[2] = 0;
		} else {
			double relVal = value - 510;
			color[0] = 255;
			color[1] = 255 - relVal;
			color[2] = 0;
		}

		for(unsigned int i = 0; i < 3; ++i) {
			color[i] /= 255;
		}

		return color;
	}

	void streamVisualization(std::function<double(unsigned int)> getVal) const {
		unsigned int size = abstraction->getAbstractionSize();

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < size; ++i) {
			double val = getVal(i);

			if(std::isinf(val)) continue;

			if(val < min) min = val;
			if(val > max) max = val;
		}

		for(unsigned int i = 0; i < size; ++i) {
			double val = getVal(i);
			if(std::isinf(val)) continue;

			auto state = abstraction->getState(i);
			auto color = getColor(min, max, val);

			streamPoint(state, color[0], color[1], color[2], 1);
		}
	}

	void generatePythonPlotting(std::function<double(unsigned int)> getVal, std::string filename) const {
		FILE *f = fopen(filename.c_str(), "w");

		unsigned int size = abstraction->getAbstractionSize();

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(unsigned int i = 0; i < size; ++i) {
			double val = getVal(i);

			if(std::isinf(val)) continue;

			if(val < min) min = val;
			if(val > max) max = val;
		}

		fprintf(stderr, "%g - %g\n", min, max);

		fprintf(f, "import numpy as np\nfrom mpl_toolkits.mplot3d import Axes3D\nimport matplotlib.pyplot as plt\n");
		fprintf(f, "fig = plt.figure()\nax = fig.add_subplot(111, projection='3d')\nax.scatter(");

		unsigned int last = size - 1;

		for(unsigned int coord = 0; coord < 4; ++coord) {
			if(coord == 3) {
				fprintf(f, "c=");
			}
			fprintf(f, "[");
			for(unsigned int i = 0; i < size; ++i) {
				double val = getVal(i);

				if(std::isinf(val)) continue;

				auto state = abstraction->getState(i)->as<ompl::base::SE3StateSpace::StateType>();

				if(coord == 0) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getX());
				} else if(coord == 1) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getY());
				} else if(coord == 2) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getZ());
				} else if(coord == 3) {
					auto color = getColor(min, max, val);
					fprintf(f, (i < last) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
				}
			}
			fprintf(f, (coord < 3) ? "]," : "], depthshade=False");
		}
		fprintf(f, ")\nplt.show()\n");
		fclose(f);
	}

	void generatePythonPlotting(std::function<double(unsigned int)> getVal, std::vector<unsigned int> &path, std::string filename) const {
		FILE *f = fopen(filename.c_str(), "w");

		unsigned int size = abstraction->getAbstractionSize();

		double min = std::numeric_limits<double>::infinity();
		double max = -std::numeric_limits<double>::infinity();
		for(auto i : path) {
			double val = getVal(i);

			if(std::isinf(val)) continue;

			if(val < min) min = val;
			if(val > max) max = val;
		}

		fprintf(stderr, "%g - %g\n", min, max);

		fprintf(f, "import numpy as np\nfrom mpl_toolkits.mplot3d import Axes3D\nimport matplotlib.pyplot as plt\n");
		fprintf(f, "fig = plt.figure()\nax = fig.add_subplot(111, projection='3d')\nax.scatter(");

		unsigned int last = size - 1 + path.size();

		for(unsigned int coord = 0; coord < 4; ++coord) {
			if(coord == 3) {
				fprintf(f, "c=");
			}
			fprintf(f, "[");
			for(unsigned int i = 0; i < size; ++i) {
				double val = getVal(i);

				if(std::isinf(val)) continue;

				auto state = abstraction->getState(i)->as<ompl::base::SE3StateSpace::StateType>();

				if(coord == 0) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getX());
				} else if(coord == 1) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getY());
				} else if(coord == 2) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getZ());
				} else if(coord == 3) {
					std::vector<double> color = {1, 1, 1};
					fprintf(f, (i < last) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
				}
			}

			for(auto i : path) {
				double val = getVal(i);

				if(std::isinf(val)) continue;

				auto state = abstraction->getState(i)->as<ompl::base::SE3StateSpace::StateType>();

				if(coord == 0) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getX());
				} else if(coord == 1) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getY());
				} else if(coord == 2) {
					fprintf(f, (i < last) ? "%g, ": "%g", state->getZ());
				} else if(coord == 3) {
					auto color = getColor(min, max, val);
					fprintf(f, (i < last) ? "(%g, %g, %g), ": "(%g, %g, %g)", color[0], color[1], color[2]);
				}
			}
			fprintf(f, (coord < 3) ? "]," : "], depthshade=False");
		}

		fprintf(f, ")\nplt.show()\n");
		fclose(f);
	}


	Abstraction *abstraction;
	StateSamplerPtr fullStateSampler;
	double stateRadius;
};

}

}
