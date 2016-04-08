#pragma once

#include "AppBase.hpp"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>

namespace ompl {
namespace app {

class StraightLinePlanning : public AppBase<CONTROL> {
public:
	class StraightLinePropagator : public ompl::control::StatePropagator {
	public:
		StraightLinePropagator(ompl::control::SpaceInformation *si) : ompl::control::StatePropagator(si) {}
		StraightLinePropagator(const ompl::control::SpaceInformationPtr &si) : ompl::control::StatePropagator(si) {}

		virtual bool steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const {
			const auto fromSE2 = from->as<ompl::base::SE2StateSpace::StateType>();
			const auto toSE2 = to->as<ompl::base::SE2StateSpace::StateType>();
			auto resultRVC = result->as<ompl::control::RealVectorControlSpace::ControlType>();

			double dx = toSE2->getX() - fromSE2->getX();
			double dy = toSE2->getY() - fromSE2->getY();

			duration = sqrt(dx*dx + dy*dy);

			resultRVC->values[0] = dx / duration;
			resultRVC->values[1] = dy / duration;

			return true;
		}

		virtual void propagate(const ompl::base::State *state, const ompl::control::Control *control, const double duration, ompl::base::State *result) const {
			const auto stateSE2 = state->as<ompl::base::SE2StateSpace::StateType>();
			const auto controlRVC = control->as<ompl::control::RealVectorControlSpace::ControlType>();
			auto resultSE2 = result->as<ompl::base::SE2StateSpace::StateType>();

			resultSE2->setX(stateSE2->getX() + controlRVC->values[0] * duration);
			resultSE2->setY(stateSE2->getY() + controlRVC->values[1] * duration);

			resultSE2->setYaw(0);
		}

		virtual bool canPropagateBackward() const {
			return true;
		}

		virtual bool canSteer() const {
			return true;
		}
	};

	StraightLinePlanning() : AppBase<CONTROL>(constructControlSpace(), Motion_2D) {
		name_ = std::string("StraightLine");
		setDefaultControlBounds();
		si_->setStatePropagator(ompl::control::StatePropagatorPtr(new StraightLinePropagator(si_)));
	}

	~StraightLinePlanning() {}

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

	virtual void setDefaultControlBounds(void) {
		base::RealVectorBounds cbounds(2);
		cbounds.low[0] = -1;
		cbounds.high[0] = 1;
		cbounds.low[1] = -1;
		cbounds.high[1] = 1;
		getControlSpace()->as<control::RealVectorControlSpace>()->setBounds(cbounds);
	}

protected:

	virtual const base::State *getGeometricComponentStateInternal(const base::State *state, unsigned int /*index*/) const {
		return state;
	}

	static control::ControlSpacePtr constructControlSpace(void) {
		return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
	}
	static base::StateSpacePtr constructStateSpace(void) {
		return base::StateSpacePtr(new base::SE2StateSpace());
	}
};
}
}