#pragma once

#include "AppBase.hpp"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/math/constants/constants.hpp>

#include "../ikfast/cyton_gamma_1500_ik.cpp"

void multiply(const std::vector<double> &m1, const std::vector<double> &m2, std::vector<double> &out) {
	std::vector<double> temp(16);
	for(unsigned int row = 0; row < 4; ++row) {
		for(unsigned int col = 0; col < 4; ++col) {
			double sum = 0;
			for(unsigned int i = 0; i < 4; i++) {
				double elem1 = m1[row * 4 + i];
				double elem2 = m2[col + 4 * i];
				sum += elem1 * elem2;
			}
			temp[row * 4 + col] = sum;
		}
	}
	for(unsigned int i = 0; i < 16; i++) out[i] = temp[i];
}

namespace ompl {
namespace app {

class RobotArmPlanning : public AppBase<CONTROL> {
public:
	class RobotArmPropagator : public ompl::control::StatePropagator {
	public:
		RobotArmPropagator(const ompl::control::SpaceInformationPtr &si, const std::vector<double> &linkLengths, double jointPadding, const std::vector<unsigned int> &rotationAxes, const ompl::base::RealVectorBounds &jointRanges) :
			ompl::control::StatePropagator(si), linkLengths(linkLengths), jointPadding(jointPadding), rotationAxes(rotationAxes), jointRanges(jointRanges) {

		}

		virtual bool steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const {
			return false;
		}

		virtual void propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
			std::vector<double> transform(16, 0);
			transform[0] = transform[5] = transform[10] = transform[15] = 1;

			const auto controlRVC = control->as<ompl::control::RealVectorControlSpace::ControlType>();

			std::vector<double> translation(16, 0);
			translation[0] = translation[5] = translation[10] = translation[15] = 1;

			for(unsigned int i = 0; i < linkLengths.size(); i++) {
				auto stateSE3 = state->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(i);
				auto resultStateSE3 = result->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(i);
				auto &so2 = stateSE3->rotation();
				double angle = 2 * acos(so2.w) +  controlRVC->values[i] * duration;

				if(angle < jointRanges.low[i]) {
					angle = jointRanges.low[i];
				}
				if(angle > jointRanges.high[i]) {
					angle = jointRanges.high[i];
				}

				if(i > 0) {
					translation[11] = linkLengths[i-1] + jointPadding;
				}

				std::vector<double> rotation(16, 0);
				rotation[0] = rotation[5] = rotation[10] = rotation[15] = 1;

				if(rotationAxes[i] == 0) { // rotate around x
					rotation[5] = cos(angle);
					rotation[6] = -sin(angle);
					rotation[9] = sin(angle);
					rotation[10] = cos(angle);
				} else if(rotationAxes[i] == 1) { //rotate around y
					rotation[0] = cos(angle);
					rotation[2] = sin(angle);
					rotation[8] = -sin(angle);
					rotation[10] = cos(angle);
				} else if(rotationAxes[i] == 2) { //rotate around z
					rotation[0] = cos(angle);
					rotation[1] = -sin(angle);
					rotation[4] = sin(angle);
					rotation[5] = cos(angle);
				}

				multiply(rotation, translation, rotation);
				multiply(transform, rotation, transform);

				getState(transform, resultStateSE3);
			}
		}

		void getState(const std::vector<double> &transform, base::SE3StateSpace::StateType *stateSE3) const {
			stateSE3->setXYZ(transform[3], transform[7], transform[11]);
			auto &so2 = stateSE3->rotation();
			so2.w = sqrt(1 + transform[0] + transform[5] + transform[10]) / 2;
			so2.x = (transform[9] - transform[6])/( 4 * so2.w);
			so2.y = (transform[2] - transform[8])/( 4 * so2.w);
			so2.z = (transform[4] - transform[1])/( 4 * so2.w);
		}

		virtual bool canPropagateBackward() const {
			return true;
		}

		virtual bool canSteer() const {
			return false;
		}

		ompl::base::RealVectorBounds jointRanges;
		std::vector<unsigned int> rotationAxes;
		std::vector<double> linkLengths;
		double jointPadding;
	};


	RobotArmPlanning(const std::vector<double> &linkLengths, double jointPadding, const std::vector<unsigned int> &rotationAxes, const ompl::base::RealVectorBounds &jointRanges) :
		AppBase<CONTROL>(constructControlSpace(linkLengths.size()), Motion_3D), timeStep(1e2), numberOfLinks(linkLengths.size()), linkLengths(linkLengths), jointPadding(jointPadding),
		rotationAxes(rotationAxes), jointRanges(jointRanges) {
		name_ = std::string("RobotArmPlanning");
		setDefaultControlBounds();
		si_->setStatePropagator(ompl::control::StatePropagatorPtr(new RobotArmPropagator(si_, linkLengths, jointPadding, rotationAxes, jointRanges)));
	}

	~RobotArmPlanning() {

	}

	bool isSelfCollisionEnabled(void) const {
		return true;
	}

	virtual unsigned int getRobotCount(void) const {
		return numberOfLinks;
	}

	virtual base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<base::CompoundStateSpace> s(getStateSpace());

		aiVector3D c = getRobotCenter(0);
		double offset = 0;
		for(unsigned int i = 0; i < numberOfLinks; i++) {
			auto stateSE3 = s->as<base::SE3StateSpace::StateType>(i);
			if(i > 0) {
				offset += linkLengths[i-1] + jointPadding;
			}
			stateSE3->setZ(offset);
		}
		return s;
	}

	base::ScopedState<> buildState(const std::vector<double> &jointAngles) const {
		base::ScopedState<base::CompoundStateSpace> s(getStateSpace());

		std::vector<double> transform(16, 0);
		transform[0] = transform[5] = transform[10] = transform[15] = 1;
		std::vector<double> translation= transform;

		aiVector3D c = getRobotCenter(0);
		double offset = 0;
		for(unsigned int i = 0; i < numberOfLinks; i++) {
			auto stateSE3 = s->as<base::SE3StateSpace::StateType>(i);


			if(i > 0) {
				translation[11] = linkLengths[i-1] + jointPadding;
			}

			std::vector<double> rotation(16, 0);
			rotation[0] = rotation[5] = rotation[10] = rotation[15] = 1;

			double angle = jointAngles[i];

			if(angle < jointRanges.low[i] || angle > jointRanges.high[i]) {
				throw ompl::Exception("Start State Joint Angle Out Of Bounds");
			}

			if(rotationAxes[i] == 0) { // rotate around x
				rotation[5] = cos(angle);
				rotation[6] = -sin(angle);
				rotation[9] = sin(angle);
				rotation[10] = cos(angle);
			} else if(rotationAxes[i] == 1) { //rotate around y
				rotation[0] = cos(angle);
				rotation[2] = sin(angle);
				rotation[8] = -sin(angle);
				rotation[10] = cos(angle);
			} else if(rotationAxes[i] == 2) { //rotate around z
				rotation[0] = cos(angle);
				rotation[1] = -sin(angle);
				rotation[4] = sin(angle);
				rotation[5] = cos(angle);
			}

			multiply(rotation, translation, rotation);
			multiply(transform, rotation, transform);

			stateSE3->setXYZ(transform[3], transform[7], transform[11]);
			auto &so2 = stateSE3->rotation();
			so2.w = sqrt(1 + transform[0] + transform[5] /*+ transform[10]*/) / 2;
			so2.x = (transform[9] - transform[6])/( 4 * so2.w);
			so2.y = (transform[2] - transform[8])/( 4 * so2.w);
			so2.z = (transform[4] - transform[1])/( 4 * so2.w);
		}
		return s;
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const {
		IkSolutionList<IkReal> solutions;

		const auto se3 = state->as<ompl::base::SE3StateSpace::StateType>();

		IkReal translation[3]= { se3->getX(),
								 se3->getY(),
								 se3->getZ()};

		const auto &rot = se3->rotation();

		IkReal rotation[9] = { 1 - 2 * rot.y * rot.y - 2 * rot.z * rot.z,
							   2 * rot.z * rot.y - 2 * rot.z * rot.w,
							   2 * rot.x * rot.z + 2 * rot.y * rot.w,
							   2 * rot.x * rot.y + 2 * rot.z * rot.w,
							   1 - 2 * rot.x * rot.x - 2 * rot.z * rot.z,
							   2 * rot.y * rot.z - 2 * rot.x * rot.w,
							   2 * rot.x * rot.z - 2 * rot.y * rot.w,
							   2 * rot.y * rot.z + 2 * rot.x * rot.w,
							   1 - 2 * rot.x * rot.x - 2 * rot.y * rot.y};

		IkReal freeJoints[3] = {0, 0, 0};

		ComputeIk(translation, rotation, freeJoints, solutions);

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

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(unsigned int index) const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace() const {
		throw ompl::Exception("RobotArmPlanning::getGeometricComponentStateSpace(void) not implemented");
	}

	virtual void setDefaultControlBounds(void) {
		base::RealVectorBounds bounds(numberOfLinks);
		bounds.setLow(-1);
		bounds.setHigh(1);
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int index) const {
		return state->as<base::CompoundState>()->components[index];
	}

	static control::ControlSpacePtr constructControlSpace(unsigned int numberOfLinks) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(numberOfLinks), numberOfLinks));
	}

	static base::StateSpacePtr constructStateSpace(unsigned int numberOfLinks) {
		base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
		for(unsigned int i = 0; i < numberOfLinks; i++) {
			stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE3StateSpace()), 1.);	
		}
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	unsigned int numberOfLinks;
	double timeStep, jointPadding;
	std::vector<double> linkLengths;
	std::vector<unsigned int> rotationAxes;
	ompl::base::RealVectorBounds jointRanges;
	mutable ompl::RNG rng;
};

}
}