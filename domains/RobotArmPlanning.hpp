#pragma once

#include "AppBase.hpp"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>

#include "../ikfast/cyton_gamma_1500_ik.cpp"

#include "kinematicchainhelpers.hpp"

namespace ompl {
namespace app {

class RobotArmPlanning : public AppBase<CONTROL> {
public:
	class RobotArmPropagator : public ompl::control::StatePropagator {
	public:
		RobotArmPropagator(const ompl::control::SpaceInformationPtr &si, const std::vector<KinematicChainHelpers::TransformPair> &transformPairs, const std::vector<std::vector<double>> &jointAxes,
			const ompl::base::RealVectorBounds &jointRanges) : ompl::control::StatePropagator(si), transformPairs(transformPairs), jointAxes(jointAxes), jointRanges(jointRanges) {
		}

		virtual bool steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const {
			return false;
		}

		virtual void propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
			const auto controlRVC = control->as<ompl::control::RealVectorControlSpace::ControlType>();

			std::vector<double> angles(jointAxes.size());
			KinematicChainHelpers::getJointAngles(state, transformPairs, jointAxes, angles);

			for(unsigned int i = 0; i < jointAxes.size(); i++) {
				angles[i] += controlRVC->values[i] * duration;

				if(angles[i] < jointRanges.low[i]) {
					angles[i] = jointRanges.low[i];
				}
				if(angles[i] > jointRanges.high[i]) {
					angles[i] = jointRanges.high[i];
				}
			}

			KinematicChainHelpers::buildState(angles, transformPairs, jointAxes, result);
		}

		virtual bool canPropagateBackward() const {
			return true;
		}

		virtual bool canSteer() const {
			return false;
		}

		std::vector<KinematicChainHelpers::TransformPair> transformPairs;
		std::vector<std::vector<double>> jointAxes;
		ompl::base::RealVectorBounds jointRanges;
	};


	RobotArmPlanning(const std::vector<std::vector<double>> &transforms, const std::vector<std::vector<double>> &jointAxes, const ompl::base::RealVectorBounds &jointRanges) :
		AppBase<CONTROL>(constructControlSpace(jointAxes.size()), Motion_3D), numberOfLinks(jointAxes.size()),
		jointAxes(jointAxes), jointRanges(jointRanges) {
		name_ = std::string("RobotArmPlanning");
		setDefaultControlBounds();

		for(const auto &transform : transforms) {
			transformPairs.emplace_back(KinematicChainHelpers::getTransformPair(transform));
		}

		si_->setStatePropagator(ompl::control::StatePropagatorPtr(new RobotArmPropagator(si_, transformPairs, jointAxes, jointRanges)));
	}

	~RobotArmPlanning() {

	}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}

	virtual unsigned int getRobotCount(void) const {
		return numberOfLinks;
	}

	virtual base::ScopedState<> getDefaultStartState(void) const {
		std::vector<double> angles(jointAxes.size());
		return buildState(angles);
	}

	base::ScopedState<> buildState(const std::vector<double> &jointAngles) const {
		base::ScopedState<> s(getStateSpace());
		KinematicChainHelpers::buildState(jointAngles, transformPairs, jointAxes, s.get());
		return s;
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const {
		IkSolutionList<IkReal> solutions;
		computeIKSolutionsFromAbstractState(state.get(), solutions);

		if(solutions.GetNumSolutions() == 0) {
			throw ompl::Exception("couldn't compute an ik solution");
		}

		unsigned int solutionIndex = rng.uniformInt(0, solutions.GetNumSolutions());

		const auto &solution = solutions.GetSolution(solutionIndex);

		std::vector<IkReal> angles, freeAngles;
		solution.GetSolution(angles, freeAngles);
		
		std::vector<double> anglesVec(GetNumJoints());
		for(unsigned int i = 0; i < GetNumJoints(); i++) {
			anglesVec[i] = angles[i];
		}

		return buildState(anglesVec);
	}

	bool checkValidAbstractState(const ompl::base::State *state) const {
		IkSolutionList<IkReal> solutions;
		computeIKSolutionsFromAbstractState(state, solutions);
		return solutions.GetNumSolutions() > 0;
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(unsigned int index) const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace() const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
	}

	virtual void setDefaultControlBounds(void) {
		base::RealVectorBounds bounds(numberOfLinks);
		bounds.setLow(-0.1);
		bounds.setHigh(0.1);
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
	}

	double getMaximumTranslationalVelocity() const {
		throw new ompl::Exception("RobotArmPlanning::getMaximumTranslationalVelocity not implemented");
	}

protected:

	void computeIKSolutionsFromAbstractState(const ompl::base::State *state, IkSolutionList<IkReal> &solutions) const {
		//we could try to cache these if this turns out to be too slow compute

		// abstract state is at this translational offset from the virtual end effector using in ik (0, 0.0075, 0.015)
		// this is because we want the abstract mesh to contain virtual end effector as well as the gripper fingers for
		// collision checking in the abstract space

		const auto se3 = state->as<ompl::base::SE3StateSpace::StateType>();

		std::vector<double> translation = { se3->getX(),
										    se3->getY(),
										    se3->getZ()};

		const auto &rot = se3->rotation();
		std::vector<double> rotation(9);
		KinematicChainHelpers::quaternionToMatrix3x3(rot, rotation);

		std::vector<double> freeJoints = {0, 0, 0};

		std::vector<double> abstractMeshOffsetTranslation = {0, -0.0075, -0.015};
		KinematicChainHelpers::rotatePoint(rot, abstractMeshOffsetTranslation, abstractMeshOffsetTranslation);

		for(unsigned int i = 0; i < 3; i++) {
			translation[i] += abstractMeshOffsetTranslation[i];
		}

		ComputeIk(translation.data(), rotation.data(), freeJoints.data(), solutions);
	}

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int index) const {
		if(index > numberOfLinks) { //abstract
			// abstract state is at this translational offset from the virtual end effector using in ik (0, 0.0075, 0.015)
			// this is because we want the abstract mesh to contain virtual end effector as well as the gripper fingers for
			// collision checking in the abstract space
			const ompl::base::SE3StateSpace::StateType *wristRoll = state->as<base::CompoundState>()->components[13]->as<ompl::base::SE3StateSpace::StateType>();
			const ompl::base::SE3StateSpace *se3StateSpace = getStateSpace()->as<base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(0);
			ompl::base::SE3StateSpace::StateType *abstract = se3StateSpace->allocState()->as<ompl::base::SE3StateSpace::StateType>();

			se3StateSpace->copyState(abstract, wristRoll);
			std::vector<double> abstractMeshOffsetTranslation = {0, 0.0075, 0.015};
			KinematicChainHelpers::rotatePoint(abstract->rotation(), abstractMeshOffsetTranslation, abstractMeshOffsetTranslation);

			abstract->setXYZ(abstract->getX() + abstractMeshOffsetTranslation[0],
							 abstract->getY() + abstractMeshOffsetTranslation[1],
							 abstract->getZ() + abstractMeshOffsetTranslation[2]);
			return abstract;
		}

		//actual state component request
		return state->as<base::CompoundState>()->components[index * 2];
	}

	static control::ControlSpacePtr constructControlSpace(unsigned int numberOfLinks) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(numberOfLinks), numberOfLinks));
	}

	static base::StateSpacePtr constructStateSpace(unsigned int numberOfLinks) {
		base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
		for(unsigned int i = 0; i < numberOfLinks; i++) {
			stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE3StateSpace()), 0.);
			stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(1)), 1.);
		}
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	unsigned int numberOfLinks;
	std::vector<KinematicChainHelpers::TransformPair> transformPairs;
	std::vector<std::vector<double>> jointAxes;
	ompl::base::RealVectorBounds jointRanges;
	mutable ompl::RNG rng;
};

}
}