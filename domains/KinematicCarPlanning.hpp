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

#ifndef OMPLAPP_KINEMATIC_CAR_PLANNING_
#define OMPLAPP_KINEMATIC_CAR_PLANNING_

#include "AppBase.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <boost/math/constants/constants.hpp>

namespace ompl {
namespace app {
/** \brief A class to facilitate planning for a generic kinematic car
    model

    The dynamics of the kinematic car are described by the following
    equations:
    \f{eqnarray*}{
    \dot x &=& u_0 \cos\theta,\\
    \dot y &=& u_0\sin\theta,\\
    \dot\theta &=& \frac{u_0}{L}\tan u_1,\f}
    where the control inputs \f$(u_0,u_1)\f$ are the translational
    velocity and the steering angle, respectively, and \f$L\f$ is the
    distance between the front and rear axle of the car (set to 1 by
    default).
*/
class KinematicCarPlanning : public AppBase<CONTROL> {
public:
	KinematicCarPlanning()
		: AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&KinematicCarPlanning::ode, this, _1, _2, _3))) {
		name_ = std::string("KinematicCar");
		setDefaultControlBounds();

		si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&KinematicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
	}


	KinematicCarPlanning(const control::ControlSpacePtr &controlSpace)
		: AppBase<CONTROL>(controlSpace, Motion_2D), timeStep_(1e-2), lengthInv_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&KinematicCarPlanning::ode, this, _1, _2, _3))) {
		setDefaultControlBounds();

		si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&KinematicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
	}

	~KinematicCarPlanning() {
	}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}
	virtual unsigned int getRobotCount(void) const {
		return 1;
	}
	virtual base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<base::SE2StateSpace> sSE2(getStateSpace());
		aiVector3D s = getRobotCenter(0);

		sSE2->setX(s.x);
		sSE2->setY(s.y);
		sSE2->setYaw(0.);
		return sSE2;
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const {
		return state;
	}
	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(void) const {
		return getStateSpace();
	}
	double getVehicleLength() {
		return 1./lengthInv_;
	}
	void setVehicleLength(double length) {
		lengthInv_ = 1./length;
	}
	virtual void setDefaultControlBounds(void) {
		base::RealVectorBounds cbounds(2);
		cbounds.low[0] = -5.;
		cbounds.high[0] = 5.;
		cbounds.low[1] = -boost::math::constants::pi<double>() * 30. / 180.;
		cbounds.high[1] = boost::math::constants::pi<double>() * 30. / 180.;
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
	}

	double getMaximumTranslationalVelocity() const {
		//this value matches the the max absolute value of cbounds.low and cbound.high for the control space from above
		return 0.5;
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int /*index*/) const {
		return state;
	}

	virtual void ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot) {
		const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

		// zero out qdot
		qdot.resize(q.size(), 0);

		qdot[0] = u[0] * cos(q[2]);
		qdot[1] = u[0] * sin(q[2]);
		qdot[2] = u[0] * lengthInv_ * tan(u[1]);
	}

	virtual void postPropagate(const base::State * /*state*/, const control::Control * /*control*/, const double /*duration*/, base::State *result) {
		// Normalize orientation value between 0 and 2*pi
		const base::SO2StateSpace *SO2 = getStateSpace()->as<base::SE2StateSpace>()
		                                 ->as<base::SO2StateSpace>(1);
		base::SO2StateSpace::StateType *so2 = result->as<base::SE2StateSpace::StateType>()
		                                      ->as<base::SO2StateSpace::StateType>(1);
		SO2->enforceBounds(so2);
	}

	static control::ControlSpacePtr constructControlSpace(void) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
	}
	static base::StateSpacePtr constructStateSpace(void) {
		return base::StateSpacePtr(new base::SE2StateSpace());
	}

	double timeStep_;
	double lengthInv_;
	control::ODESolverPtr odeSolver;
};
}
}

#endif