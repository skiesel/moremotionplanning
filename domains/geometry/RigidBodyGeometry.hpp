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

#ifndef OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_
#define OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_

#include "../config.hpp"
#include "GeometrySpecification.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#if OMPL_HAS_PQP
#include "detail/PQPStateValidityChecker.hpp"
#endif

#include "detail/FCLStateValidityChecker.hpp"


#if OMPL_HAS_ASSIMP3
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#else
#include <assimp/aiScene.h>
#include <assimp/assimp.hpp>
#endif

#include <string>
#include <vector>

namespace ompl {

/** \brief Namespace containing code specific to OMPL.app */
namespace app {
/** \brief Enumeration of the possible collision checker types */
enum CollisionChecker
{ PQP, FCL };

class RigidBodyGeometry {
public:

	/** \brief Constructor expects a state space that can represent a rigid body */
	/// \param mtype The motion model (2D or 3D) for the rigid body.
	/// \param ctype The type of collision checker to use for rigid body planning.
	explicit
	RigidBodyGeometry(MotionModel mtype, CollisionChecker ctype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(ctype) {
	}

	/// \brief Constructor expects a state space that can represent a rigid body
	/// \param mtype The motion model (2D or 3D) for the rigid body.
	/// \remarks This constructor defaults to a PQP state validity checker
	explicit
	RigidBodyGeometry(MotionModel mtype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(FCL) {
	}

	virtual ~RigidBodyGeometry(void) {
	}

	MotionModel getMotionModel(void) const {
		return mtype_;
	}

	CollisionChecker getCollisionCheckerType(void) const {
		return ctype_;
	}

	bool hasEnvironment(void) const {
		return !importerEnv_.empty();
	}

	bool hasRobot(void) const {
		return !importerRobot_.empty();
	}

	unsigned int getLoadedRobotCount(void) const {
		return importerRobot_.size();
	}

	/** \brief Get the robot's center (average of all the vertices of all its parts) */
	aiVector3D getRobotCenter(unsigned int robotIndex) const {
		aiVector3D s(0.0, 0.0, 0.0);
		if(robotIndex >= importerRobot_.size())
			throw Exception("Robot " + boost::lexical_cast<std::string>(robotIndex) + " not found.");

		scene::sceneCenter(importerRobot_[robotIndex]->GetScene(), s);
		return s;
	}

	/** \brief This function specifies the name of the CAD
	    file representing the environment (\e
	    env). Returns 1 on success, 0 on failure. */
	virtual bool setEnvironmentMesh(const std::string &env) {
		importerEnv_.clear();
		computeGeometrySpecification();
		return addEnvironmentMesh(env);
	}


	/** \brief This function specifies the name of the CAD
	    file representing a part of the environment (\e
	    env). Returns 1 on success, 0 on failure. */
	virtual bool addEnvironmentMesh(const std::string &env) {
		assert(!env.empty());
		std::size_t p = importerEnv_.size();
		importerEnv_.resize(p + 1);
		importerEnv_[p].reset(new Assimp::Importer());

		const aiScene *envScene = importerEnv_[p]->ReadFile(env.c_str(),
		                          aiProcess_Triangulate            |
		                          aiProcess_JoinIdenticalVertices  |
		                          aiProcess_SortByPType            |
		                          aiProcess_OptimizeGraph          |
		                          aiProcess_OptimizeMeshes);
		if(envScene) {
			if(!envScene->HasMeshes()) {
				OMPL_ERROR("There is no mesh specified in the indicated environment resource: %s", env.c_str());
				importerEnv_.resize(p);
			}
		} else {
			OMPL_ERROR("Unable to load environment scene: %s", env.c_str());
			importerEnv_.resize(p);
		}

		if(p < importerEnv_.size()) {
			computeGeometrySpecification();
			return true;
		} else
			return false;
	}

	/** \brief This function specifies the name of the CAD
	    file representing the robot (\e robot). Returns 1 on success, 0 on failure. */
	virtual bool setRobotMesh(const std::string &robot) {
		importerRobot_.clear();
		computeGeometrySpecification();
		return addRobotMesh(robot);
	}

	/** \brief This function specifies the name of the CAD
	   file representing a part of the robot (\e robot). Returns 1 on success, 0 on failure. */
	virtual bool addRobotMesh(const std::string &robot) {
		assert(!robot.empty());
		std::size_t p = importerRobot_.size();
		importerRobot_.resize(p + 1);
		importerRobot_[p].reset(new Assimp::Importer());

		const aiScene *robotScene = importerRobot_[p]->ReadFile(robot.c_str(),
		                            aiProcess_Triangulate            |
		                            aiProcess_JoinIdenticalVertices  |
		                            aiProcess_SortByPType            |
		                            aiProcess_OptimizeGraph          |
		                            aiProcess_OptimizeMeshes);
		if(robotScene) {
			if(!robotScene->HasMeshes()) {
				OMPL_ERROR("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
				importerRobot_.resize(p);
			}
		} else {
			OMPL_ERROR("Unable to load robot scene: %s", robot.c_str());
			importerRobot_.resize(p);
		}

		if(p < importerRobot_.size()) {
			computeGeometrySpecification();
			return true;
		} else
			return false;
	}

	/** \brief Change the type of collision checking for the rigid body */
	virtual void setStateValidityCheckerType(CollisionChecker ctype) {
		if(ctype != ctype_) {
			ctype_ = ctype;
			if(validitySvc_) {
				validitySvc_.reset();
			}

			assert(!validitySvc_);
		}
	}

	/** \brief Allocate default state validity checker using PQP. */
	const ompl::base::StateValidityCheckerPtr &allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision) {
		if(validitySvc_)
			return validitySvc_;

		GeometrySpecification geom = getGeometrySpecification();

		switch(ctype_) {
#if OMPL_HAS_PQP
		case PQP:
			if(mtype_ == Motion_2D)
				validitySvc_.reset(new PQPStateValidityChecker<Motion_2D>(si, geom, se, selfCollision));
			else
				validitySvc_.reset(new PQPStateValidityChecker<Motion_3D>(si, geom, se, selfCollision));
			break;
#endif
		case FCL:
			if(mtype_ == Motion_2D)
				validitySvc_.reset(new FCLStateValidityChecker<Motion_2D>(si, geom, se, selfCollision));
			else
				validitySvc_.reset(new FCLStateValidityChecker<Motion_3D>(si, geom, se, selfCollision));
			break;

		default:
			OMPL_ERROR("Unexpected collision checker type (%d) encountered", ctype_);
		};

		return validitySvc_;
	}

	const ompl::app::GeometrySpecification &getGeometrySpecification(void) const {
		return geom_;
	}

	/** \brief The bounds of the environment are inferred
	    based on the axis-aligned bounding box for the objects
	    in the environment. The inferred size is multiplied by
	    \e factor. By default \e factor = 1, */
	void setBoundsFactor(double factor) {
		factor_ = factor;
	}

	/** \brief Get the data set by setBoundsFactor() */
	double getBoundsFactor(void) const {
		return factor_;
	}

	/** \brief The bounds of the environment are inferred
	    based on the axis-aligned bounding box for the objects
	    in the environment. \e add is added to the inferred
	    size. By default \e add = 0, */
	void setBoundsAddition(double add) {
		add_ = add;
	}

	/** \brief Get the data set by setBoundsAddition() */
	double getBoundsAddition(void) const {
		return add_;
	}

	/** \brief Given the representation of an environment,
	    infer its bounds. The bounds will be 2-dimensional
	    when planning in 2D and 3-dimensional when planning in
	    3D. */
	ompl::base::RealVectorBounds inferEnvironmentBounds(void) const {
		base::RealVectorBounds bounds(3);

		for(unsigned int i = 0 ; i < importerEnv_.size() ; ++i) {
			std::vector<aiVector3D> vertices;
			scene::extractVertices(importerEnv_[i]->GetScene(), vertices);
			scene::inferBounds(bounds, vertices, factor_, add_);
		}

		if(mtype_ == Motion_2D) {
			bounds.low.resize(2);
			bounds.high.resize(2);
		}

		return bounds;
	}

protected:

	void computeGeometrySpecification(void) {
		validitySvc_.reset();
		geom_.obstacles.clear();
		geom_.obstaclesShift.clear();
		geom_.robot.clear();
		geom_.robotShift.clear();

		for(unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
			geom_.obstacles.push_back(importerEnv_[i]->GetScene());

		for(unsigned int i = 0 ; i < importerRobot_.size() ; ++i) {
			geom_.robot.push_back(importerRobot_[i]->GetScene());
			aiVector3D c = getRobotCenter(i);
			if(mtype_ == Motion_2D)
				c[2] = 0.0;
			geom_.robotShift.push_back(c);
		}
	}

	MotionModel         mtype_;

	/** \brief The factor to multiply inferred environment bounds by (default 1) */
	double              factor_;

	/** \brief The value to add to inferred environment bounds (default 0) */
	double              add_;

	/** \brief Instance of assimp importer used to load environment */
	std::vector< boost::shared_ptr<Assimp::Importer> > importerEnv_;

	/** \brief Instance of assimp importer used to load robot */
	std::vector< boost::shared_ptr<Assimp::Importer> > importerRobot_;

	/** \brief Object containing mesh data for robot and environment */
	GeometrySpecification         geom_;

	/** \brief Instance of the state validity checker for collision checking */
	base::StateValidityCheckerPtr validitySvc_;

	/** \brief Value containing the type of collision checking to use */
	CollisionChecker              ctype_;

};

}
}

#endif
