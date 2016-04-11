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

#ifndef OMPLAPP_BLIMP_PLANNING_
#define OMPLAPP_BLIMP_PLANNING_

#include "AppBase.hpp"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl {
namespace app {
/** \brief A class to facilitate planning for a simple blimp model

    The dynamics of the blimp are described by the following equations:
    \f{eqnarray*}{
    \ddot x &=& u_f\cos\theta,\\
    \ddot y &=& u_f\sin\theta,\\
    \ddot z &=& u_z,\\
    \ddot\theta &=& u_\theta,\f}
    where \f$(x,y,z)\f$ is the position, \f$\theta\f$ the heading, and the
    controls \f$(u_f,u_z,u_\theta)\f$ control their rate of change.
*/
class BlimpPlanning : public AppBase<CONTROL> {
public:
	BlimpPlanning()
		: AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&BlimpPlanning::ode, this, _1, _2, _3))) {
		name_ = std::string("Blimp");
		setDefaultBounds();

		si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&BlimpPlanning::postPropagate, this, _1, _2, _3, _4)));
	}
	~BlimpPlanning() {
	}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}
	virtual unsigned int getRobotCount(void) const {
		return 1;
	}
	virtual ompl::base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<base::SE3StateSpace> s(getGeometricComponentStateSpace());
		aiVector3D c = getRobotCenter(0);

		s->setXYZ(c.x, c.y, c.z);
		s->rotation().setIdentity();
		return getFullStateFromGeometricComponent(s);
	}

	virtual ompl::base::ScopedState<> getFullStateFromGeometricComponent(
	    const base::ScopedState<> &state) const {
		const base::SO3StateSpace::StateType &rot = state->as<base::SE3StateSpace::StateType>()->rotation();
		double norm = sqrt(rot.z * rot.z + rot.w * rot.w);
		base::ScopedState<> s(getStateSpace());
		base::SO3StateSpace::StateType &rot2 =
		    s->as<base::CompoundStateSpace::StateType>()->as<base::SE3StateSpace::StateType>(0)->rotation();
		s = 0.0;
		// set x,y,z
		for(unsigned int i=0; i<3; ++i)
			s[i] = state[i];
		// set heading
		rot2.z = rot.z / norm;
		rot2.w = rot.w / norm;
		return s;
	}


	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(void) const {
		return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
	}

	virtual void setDefaultBounds() {
		base::RealVectorBounds velbounds(3), omegabounds(1), controlbounds(3);

		velbounds.setLow(-1);
		velbounds.setHigh(1);
		getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(1)->setBounds(velbounds);
		omegabounds.setLow(-.2);
		omegabounds.setHigh(.2);
		getStateSpace()->as<base::CompoundStateSpace>()->as<base::RealVectorStateSpace>(2)->setBounds(omegabounds);
		controlbounds.setLow(-1);
		controlbounds.setHigh(1);
		controlbounds.setLow(2,-.3);
		controlbounds.setHigh(2,.3);
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(controlbounds);

	}

	double getMaximumTranslationalVelocity() const {
		// Maximum is 1 in x/y and 1 in z
		return sqrt(2);
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int /*index*/) const {
		return state->as<base::CompoundState>()->components[0];
	}

	void postPropagate(const base::State * /*state*/, const control::Control * /*control*/, const double /*duration*/, base::State *result) {
		// Constrain orientation of the blimp about the z axis.
		base::CompoundStateSpace::StateType &s = *result->as<base::CompoundStateSpace::StateType>();
		base::SE3StateSpace::StateType &pose = *s.as<base::SE3StateSpace::StateType>(0);
		pose.rotation().setAxisAngle(0,0,1, pose.rotation().x);

		// Enforce velocity bounds
		getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(1)->enforceBounds(s[1]);

		// Enforce steering bounds
		getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(2)->enforceBounds(s[2]);
	}

	virtual void ode(const control::ODESolver::StateType &q, const control::Control *ctrl, control::ODESolver::StateType &qdot) {
		// Retrieving control inputs
		const double *u = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;

		// zero out qdot
		qdot.resize(q.size(), 0);

		qdot[0] = q[7];
		qdot[1] = q[8];
		qdot[2] = q[9];

		qdot[3] = q[10];

		qdot[7] = u[0] * cos(q[3]);
		qdot[8] = u[0] * sin(q[3]);
		qdot[9] = u[1];
		qdot[10] = u[2];
	}

	static control::ControlSpacePtr constructControlSpace(void) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 3));
	}
	static ompl::base::StateSpacePtr constructStateSpace(void) {
		base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());

		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE3StateSpace()), 1.);
		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(3)), .3);
		stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(1)), .3);
		stateSpace->as<base::CompoundStateSpace>()->lock();
		return stateSpace;
	}

	double timeStep_;
	control::ODESolverPtr odeSolver;
};

}
}

#endif
