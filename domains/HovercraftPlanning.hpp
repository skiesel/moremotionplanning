#pragma once

#include "AppBase.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>

namespace ompl {
namespace app {

class HovercraftPlanning : public AppBase<CONTROL> {
public:
	// Controllability of a Hovercraft with Unilateral Thrusters?
	// http://msl.cs.uiuc.edu/~lavalle/cs476_1998/projects/marlow/

	HovercraftPlanning()
		: AppBase<CONTROL>(constructControlSpace(), Motion_2D), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&HovercraftPlanning::ode, this, _1, _2, _3))) {
		name_ = std::string("Hovercraft");

		mass = 1;
		radius = 1;
		translational_friction = 0.0;
		rotational_friction = 0.0;
		timeStep = 0.01;

		setDefaultBounds();
		si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&HovercraftPlanning::postPropagate, this, _1, _2, _3, _4)));
	}

	~HovercraftPlanning() {}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}
	virtual unsigned int getRobotCount(void) const {
		return 1;
	}
	virtual base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<base::CompoundStateSpace> s(getStateSpace());
		base::SE2StateSpace::StateType &pose = *s->as<base::SE2StateSpace::StateType>(0);
		base::RealVectorStateSpace::StateType &vel = *s->as<base::RealVectorStateSpace::StateType>(1);

		aiVector3D c = getRobotCenter(0);
		pose.setX(c.x);
		pose.setY(c.y);
		pose.setYaw(0.);
		vel.values[0] = 0.;
		vel.values[1] = 0.;
		vel.values[2] = 0.;
		return s;
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const {
		base::ScopedState<> r(si_);
		r = 0.0;
		r[0] = state[0];
		r[1] = state[1];
		r[2] = state[2];
		return r;
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(void) const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
	}

	virtual void setDefaultBounds() {
		base::RealVectorBounds bounds(3);
		bounds.low[0] = -1.;
		bounds.high[0] = 1.;
		bounds.low[1] = -1.;
		bounds.high[1] = 1.;
		bounds.low[2] = -boost::math::constants::pi<double>() * 60. / 180.;
		bounds.high[2] = boost::math::constants::pi<double>() * 60. / 180.;
		getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(bounds);

		bounds = base::RealVectorBounds(2);
		bounds.low[0] = 0.0;
		bounds.high[0] = 1.0;
		bounds.low[1] = -1.0;
		bounds.high[1] = 1.0;
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
	}

	double getMaximumTranslationalVelocity() const {
		// Maximum is 1 in x and 1 in y
		return sqrt(2);
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int /*index*/) const {
		return state->as<base::CompoundState>()->components[0];
	}

	virtual void ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot) {
		// Retrieving control inputs
		const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

		// zero out qdot
		qdot.resize(q.size(), 0);

		qdot[0] = q[3];
		qdot[1] = q[4];
		qdot[2] = q[5];

		qdot[3] = (u[0] / mass) * cos(q[2]) - (translational_friction / mass) * q[3];
		qdot[4] = (u[0] / mass) * sin(q[2]) - (translational_friction / mass) * q[4];
		qdot[5] = u[1] / (0.5 * mass * radius * radius) - (rotational_friction / mass) * q[5];
	}


	virtual void postPropagate(const base::State * /*state*/, const control::Control * /*control*/, const double /*duration*/, base::State *result) {
		// Normalize orientation value between 0 and 2*pi
		const base::SO2StateSpace *SO2 = getStateSpace()->as<base::CompoundStateSpace>()
		                                 ->as<base::SE2StateSpace>(0)->as<base::SO2StateSpace>(1);
		base::SO2StateSpace::StateType *so2 = result->as<base::CompoundStateSpace::StateType>()
		                                      ->as<base::SE2StateSpace::StateType>(0)->as<base::SO2StateSpace::StateType>(1);
		SO2->enforceBounds(so2);
	}


	static control::ControlSpacePtr constructControlSpace(void) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
	}
	static base::StateSpacePtr constructStateSpace(void) {
		base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE2StateSpace()), 1.);
		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), .3);
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	double timeStep, mass, radius, translational_friction, rotational_friction;
	control::ODESolverPtr odeSolver;
};

}
}