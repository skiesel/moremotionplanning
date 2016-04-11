/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_DYNAMIC_CAR_PLANNING_

#include "AppBase.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>

namespace ompl {
namespace app {
/** \brief A class to facilitate planning for a generic second-order
    car model

    The dynamics of the second-order car are described by the following
    equations:
    \f{eqnarray*}{
    \dot x &=& v\cos\theta,\\
    \dot y &=& v\sin\theta,\\
    \dot\theta &=& \frac{vm}{L}\tan \phi,\\
    \dot v &=& u_0,\\
    \dot\phi &=& u_1,\f}
    where \f$v\f$ is the speed, \f$\phi\f$ the steering angle, the
    controls \f$(u_0,u_1)\f$ control their rate of change, \f$m\f$ is
    the mass of the car, and \f$L\f$ is the distance between the front
    and rear axle of the car. Both \f$m\f$ and \f$L\f$ are set to 1 by
    default.
*/
class DynamicCarPlanning : public AppBase<CONTROL> {
public:
	DynamicCarPlanning()
		: AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), mass_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&DynamicCarPlanning::ode, this, _1, _2, _3))) {
		name_ = std::string("DynamicCar");
		setDefaultBounds();

		si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&DynamicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
	}

	~DynamicCarPlanning() {
	}

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

	double getVehicleLength() {
		return 1./lengthInv_;
	}
	void setVehicleLength(double length) {
		lengthInv_ = 1./length;
	}
	double getMass() {
		return mass_;
	}
	void setMass(double mass) {
		mass_ = mass;
	}
	virtual void setDefaultBounds() {
		base::RealVectorBounds bounds(2);
		bounds.low[0] = -1.;
		bounds.high[0] = 1.;

//These used to be "* 30." but I needed the car to be able to turn more sharply for these maps
		bounds.low[1] = -boost::math::constants::pi<double>() * 60. / 180.;
		bounds.high[1] = boost::math::constants::pi<double>() * 60. / 180.;
		getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(bounds);
		bounds.low[0] = -.5;
		bounds.high[0] = .5;

//These used to be "* 2." but I needed the car to be able to turn more sharply for these maps
		bounds.low[1] = -boost::math::constants::pi<double>() * 8. / 180.;
		bounds.high[1] = boost::math::constants::pi<double>() * 8. / 180.;
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(bounds);
	}

	double getMaximumTranslationalVelocity() const {
		//this value matches the the max absolute value of bounds.low and bound.high for the state space from above
		return 1;
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

		qdot[0] = q[3] * cos(q[2]);
		qdot[1] = q[3] * sin(q[2]);
		qdot[2] = q[3] * mass_ * lengthInv_ * tan(q[4]);

		qdot[3] = u[0];
		qdot[4] = u[1];
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
		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(2)), .3);
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	double timeStep_;
	double lengthInv_;
	double mass_;
	control::ODESolverPtr odeSolver;
};

}
}

#endif