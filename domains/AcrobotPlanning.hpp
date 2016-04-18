#pragma once

#include "AppBase.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/RandomNumbers.h>

#include "kinematicchainhelpers.hpp"

namespace ompl {
namespace app {

class AcrobotPlanning : public AppBase<CONTROL> {
		// I think for now it's fine and we'll just steal parameters from this paper:
		// Boon 1997. Minimum-time Control of the Acrobot
		double torque_min = -1;
		double torque_max = 1;
		double tc = 0.2;


public:
	class AcrobotPropagator : public ompl::control::StatePropagator {
	public:
		AcrobotPropagator(const ompl::control::SpaceInformationPtr &si) : ompl::control::StatePropagator(si) {}

		virtual bool steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const {
			return false;
		}

		std::vector<std::vector<double>> getJointAnglesAndVelocities(const ompl::base::State *state) const {
			const auto *compound = state->as<ompl::base::CompoundStateSpace::StateType>();
			const auto *rvs1 = compound->as<ompl::base::RealVectorStateSpace::StateType>(1);
			const auto *rvs2 = compound->as<ompl::base::RealVectorStateSpace::StateType>(3);

			std::vector<std::vector<double>> angles = {
				{rvs1->values[0], rvs1->values[1]},
				{rvs2->values[0], rvs2->values[1]},
			};

			return angles;
		}

		void buildState(ompl::base::State *state, const std::vector<std::vector<double>> &anglesAndVelocities) const {
			auto *compound = state->as<ompl::base::CompoundStateSpace::StateType>();

			auto *se21 = compound->as<ompl::base::SE2StateSpace::StateType>(0);
			se21->setXY(0, 0);
			se21->setYaw(anglesAndVelocities[0][0]);

			auto *rvs1 = compound->as<ompl::base::RealVectorStateSpace::StateType>(1);
			rvs1->values[0] = normalizeTheta(anglesAndVelocities[0][0]);
			rvs1->values[1] = anglesAndVelocities[0][1];

			double endX1 = cos(anglesAndVelocities[0][0]) * l1;
			double endY1 = sin(anglesAndVelocities[0][0]) * l1;

			auto *se22 = compound->as<ompl::base::SE2StateSpace::StateType>(2);
			se22->setXY(endX1, endY1);
			se22->setYaw(anglesAndVelocities[0][0] + anglesAndVelocities[1][0]);

			auto *rvs2 = compound->as<ompl::base::RealVectorStateSpace::StateType>(3);
			rvs2->values[0] = normalizeTheta(anglesAndVelocities[1][0]);
			rvs2->values[1] = anglesAndVelocities[1][1];
		}

		virtual void propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
			const auto *rvc = control->as<ompl::control::RealVectorControlSpace::ControlType>();
			double torque = rvc->values[0];

			auto anglesAndVelocities = getJointAnglesAndVelocities(state);
			double theta1 = anglesAndVelocities[0][0];
			double theta2 = anglesAndVelocities[1][0];
			double dtheta1 = anglesAndVelocities[0][1];
			double dtheta2 = anglesAndVelocities[1][1];

			unsigned int steps = duration / ti;

			for(unsigned int i = 0; i < steps; i++) {
				double d11 = m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(theta2)) + I1 + I2;
				double d22 = m2 * lc2 * lc2 + I2;
				double d12 = m2 * (lc2 * lc2 + l1 * lc2 * cos(theta2)) + I2;
				double d21 = d12;

				double c1 = -m2 * lc1 * lc2 * dtheta2 * dtheta2 * sin(theta2) - 2 * m2 * l1 * lc2 * dtheta1 * dtheta2 * sin(theta2);
				double c2 = m2 * l1 * lc2 * dtheta1 * dtheta1 * sin(theta2);

				double phi1 = (m1 * lc1 + m2 * l1) * g * cos(theta1) + m2 * lc2 * g * cos(theta1 + theta2);
				double phi2 = m2 * lc2 * g * cos(theta1 + theta2);

				double ddtheta2 = (d11 * (torque - c2 - phi2) + d12 * (c1 + phi1)) / (d11 * d22 - d12 * d12);
				double ddtheta1 = (d12 * ddtheta2 + c1 + phi1) / -d11;

				theta1 += dtheta1 * ti;
				theta2 += dtheta2 * ti;
				dtheta1 += ddtheta1 * ti;
				dtheta2 += ddtheta2 * ti;
			}

			anglesAndVelocities[0][0] = theta1;
			anglesAndVelocities[1][0] = theta2;
			anglesAndVelocities[0][1] = dtheta1;
			anglesAndVelocities[1][1] = dtheta2;

			buildState(result, anglesAndVelocities);
		}

		inline double normalizeTheta(double t) const {
			return (t - 2 * M_PI * std::floor((t + M_PI) / (2 * M_PI)));
		}

		virtual bool canPropagateBackward() const {
			return true;
		}

		virtual bool canSteer() const {
			return false;
		}

		// I think for now it's fine and we'll just steal parameters from this paper:
		// Boon 1997. Minimum-time Control of the Acrobot

		double g = 9.81;

		double l1 = 1;
		double l2 = 1;
		double lc1 = 0.5;
		double lc2 = 0.5;
		double m1 = 10;
		double m2 = 10;
		double I1 = 1;
		double I2 = 1;
		double ti = 0.05;
	};


	AcrobotPlanning() : AppBase<CONTROL>(constructControlSpace(), Motion_2D) {
		name_ = std::string("AcrobotPlanning");
		setDefaultControlBounds();
		propagator = new AcrobotPropagator(si_);
		si_->setStatePropagator(ompl::control::StatePropagatorPtr(propagator));
		si_->setPropagationStepSize(tc);
	}

	~AcrobotPlanning() {}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}

	virtual unsigned int getRobotCount(void) const {
		return 2;
	}

	virtual base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<> state(si_);
		std::vector<std::vector<double>> angles = {
			{-M_PI / 2, 0},
			{0, 0},
		};
		propagator->buildState(state.get(), angles);
		return state;
	}

	base::ScopedState<> getDefaultGoalState(void) const {
		base::ScopedState<> state(si_);
		std::vector<std::vector<double>> angles = {
			{M_PI / 2, 0},
			{0, 0},
		};
		propagator->buildState(state.get(), angles);
		return state;
	}

	void buildState(ompl::base::State *state, const std::vector<std::vector<double>> &angles) const {
		propagator->buildState(state, angles);
	}

	static void printDrawableState(const ompl::base::State *state) {
		const auto *s = state->as<ompl::base::CompoundStateSpace::StateType>();
		const auto *rvs1 = s->as<ompl::base::RealVectorStateSpace::StateType>(1);
		const auto *rvs2 = s->as<ompl::base::RealVectorStateSpace::StateType>(3);
		fprintf(stderr, "%g, %g,\n", rvs1->values[0], rvs2->values[0]);
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &s) const {
		//we know the position of the second link, we can probably figure out *a* position of the first link
		const ompl::base::SE2StateSpace::StateType *se2In = s->as<ompl::base::SE2StateSpace::StateType>();

		ompl::base::ScopedState<> state(si_);
		ompl::base::CompoundStateSpace::StateType *cs = state.get()->as<ompl::base::CompoundStateSpace::StateType>();
		ompl::base::SE2StateSpace::StateType *se2Out = cs->as<ompl::base::SE2StateSpace::StateType>(2);
		se2Out->setXY(se2In->getX(), se2In->getY());
		se2Out->setYaw(se2In->getYaw());

		se2Out = cs->as<ompl::base::SE2StateSpace::StateType>(0);
		se2Out->setXY(0, 0);
		se2Out->setYaw(atan2(se2In->getY(), se2In->getX()));

		ompl::base::RealVectorStateSpace::StateType *rvs = cs->as<ompl::base::RealVectorStateSpace::StateType>(1);
		rvs->values[0] = se2Out->getYaw();
		rvs->values[1] = 0;

		rvs = cs->as<ompl::base::RealVectorStateSpace::StateType>(3);
		rvs->values[0] = se2In->getYaw() - se2Out->getYaw();
		rvs->values[1] = 0;

		return state;
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(unsigned int index) const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index * 2);
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace() const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
	}

	virtual void setDefaultControlBounds(void) {
		base::RealVectorBounds bounds(1);
		bounds.setLow(torque_min);
		bounds.setHigh(torque_max);
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
	}

	double getMaximumTranslationalVelocity() const {
		throw new ompl::Exception("AcrobotPlanning::getMaximumTranslationalVelocity not implemented");
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int index) const {
		if(index > 1) { //abstract
			return state->as<base::CompoundStateSpace::StateType>()->components[2];;
		}

		//actual state component request
		return state->as<base::CompoundStateSpace::StateType>()->components[index * 2];
	}

	static control::ControlSpacePtr constructControlSpace() {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 1));
	}

	static base::StateSpacePtr constructStateSpace() {
		base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
		for(unsigned int i = 0; i < 2; i++) {
			stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE2StateSpace()), 0.);
			stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(2)), 1.);
		}
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	mutable ompl::RNG rng;
	AcrobotPropagator *propagator;
};

}
}