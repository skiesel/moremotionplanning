/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_SE2_RIGID_BODY_PLANNING_

#include "AppBase.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ompl {
namespace app {

/** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
    for rigid bodies in SE2. */
class SE2RigidBodyPlanning : public AppBase<GEOMETRIC> {
public:

	SE2RigidBodyPlanning(void) : AppBase<GEOMETRIC>(base::StateSpacePtr(new base::SE2StateSpace()), Motion_2D) {
		name_ = "Rigid body planning (2D)";
	}

	virtual ~SE2RigidBodyPlanning(void) {
	}

	bool isSelfCollisionEnabled(void) const {
		return false;
	}

	virtual base::ScopedState<> getDefaultStartState(void) const {
		base::ScopedState<base::SE2StateSpace> st(getGeometricComponentStateSpace());
		aiVector3D s = getRobotCenter(0);

		st->setX(s.x);
		st->setY(s.y);
		st->setYaw(0.0);

		return st;
	}

	virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const {
		return state;
	}

	virtual const base::StateSpacePtr &getGeometricComponentStateSpace(void) const {
		return getStateSpace();
	}

	virtual unsigned int getRobotCount(void) const {
		return 1;
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int /*index*/) const {
		return state;
	}

};

}
}

#endif