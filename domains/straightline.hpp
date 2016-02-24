#include "StraightLinePlanning.hpp"

BenchmarkData straightLineBenchmark(const FileMap &params) {
	ompl::app::StraightLinePlanning *straightLine = new ompl::app::StraightLinePlanning();

	globalParameters.globalAppBaseControl = straightLine;

	ompl::app::SE2RigidBodyPlanning *abstract = new ompl::app::SE2RigidBodyPlanning();

	globalParameters.globalAbstractAppBaseGeometric = abstract;

	ompl::control::SimpleSetupPtr straightLinePtr(straightLine);

	ompl::base::StateSpacePtr stateSpace(straightLinePtr->getStateSpace());

	if(params.exists("EnvironmentBounds")) {
		auto boundsVals = params.doubleList("EnvironmentBounds");
		std::vector<double> low, high;
		for(unsigned int i = 0; i < boundsVals.size(); i++) {
			low.emplace_back(boundsVals[i++]);
			high.emplace_back(boundsVals[i]);
		}

		// set the bounds for the R^2
		ompl::base::RealVectorBounds bounds(2);
		for(unsigned int i = 0; i < 2; i++) {
			bounds.setLow(i, low[i]);
			bounds.setHigh(i, high[i]);
		}

		stateSpace->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
		abstract->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
	} else {
		OMPL_WARN("using default environment bounds");
	}

	// define start state
	ompl::base::ScopedState<ompl::base::SE2StateSpace> start(straightLine->getGeometricComponentStateSpace());
	auto startLoc = params.doubleList("Start");
	
	start->setXY(startLoc[0], startLoc[1]);
	start->setYaw(0);

	// define goal state
	ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(straightLine->getGeometricComponentStateSpace());
	auto goalLoc = params.doubleList("Goal");

	goal->setXY(goalLoc[0], goalLoc[1]);
	goal->setYaw(0);

	double goalRadius = params.doubleVal("GoalRadius");
	
	// set the start & goal states
	straightLinePtr->setStartAndGoalStates(start, goal, goalRadius);

	abstract->setStartAndGoalStates(start, goal, goalRadius);

	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	std::string homeDirString(homedir);

	auto agentMesh = params.stringVal("AgentMesh");
	auto environmentMesh = params.stringVal("EnvironmentMesh");

	straightLine->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	straightLine->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	abstract->setRobotMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + agentMesh);
	abstract->setEnvironmentMesh(homeDirString + "/gopath/src/github.com/skiesel/moremotionplanning/models/" + environmentMesh);

	if(params.exists("MinControlDuration")) {
		straightLinePtr->getSpaceInformation()->setMinMaxControlDuration(params.integerVal("MinControlDuration"), params.integerVal("MaxControlDuration"));
	}

	if(params.exists("PropagationStepSize")) {
		straightLinePtr->getSpaceInformation()->setPropagationStepSize(params.doubleVal("PropagationStepSize"));
	}

	if(!straightLinePtr->getSpaceInformation()->getStatePropagator()->canSteer()) //if it can steer, leave it alone!
		straightLinePtr->getSpaceInformation()->setDirectedControlSamplerAllocator(directedControlSamplerAllocator);

	straightLinePtr->setup();

	abstract->setup();

	globalParameters.abstractBounds = abstract->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();

	globalParameters.copyVectorToAbstractState = [](ompl::base::State *s, const std::vector<double> &values) {
		ompl::base::SE2StateSpace::StateType *state = s->as<ompl::base::SE2StateSpace::StateType>();
		state->setXY(values[0], values[1]);
		state->setYaw(0);
	};

	globalParameters.copyAbstractStateToVector = [](std::vector<double> &values, const ompl::base::State *s) {
		const ompl::base::SE2StateSpace::StateType *state = s->as<ompl::base::SE2StateSpace::StateType>();
		values.resize(3);
		values[0] = state->getX();
		values[1] = state->getY();
	};

	BenchmarkData data;
	data.benchmark = new ompl::tools::Benchmark(*straightLinePtr, straightLine->getName());
	data.simplesetup = straightLinePtr;
	data.decomposition = straightLine->allocDecomposition();

	return data;
}