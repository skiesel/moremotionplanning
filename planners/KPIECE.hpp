/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_PLANNERS_KPIECE_KPIECELOCAL_
#define OMPL_CONTROL_PLANNERS_KPIECE_KPIECELOCAL_

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cassert>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include <vector>
#include <set>

namespace ompl {

namespace control {

/// @cond IGNORE
// this is the factor by which distances are inflated when considered for addition to closest samples
static const double CLOSE_MOTION_DISTANCE_INFLATION_FACTOR = 1.1;
/// @endcond

/// @cond IGNORE
// this is the offset added to estimated distances to the goal, so we avoid division by 0
static const double DISTANCE_TO_GOAL_OFFSET = 1e-3;
/// @endcond

/**
   @anchor cKPIECELocal
   @par Short description
   KPIECE is a tree-based planner that uses a discretization
   (multiple levels, in general) to guide the exploration of
   the continuous space. This implementation is a simplified
   one, using a single level of discretization: one grid. The
   grid is imposed on a projection of the state space. When
   exploring the space, preference is given to the boundary of
   this grid. The boundary is computed to be the set of grid
   cells that have less than 2n non-diagonal neighbors in an
   n-dimensional projection space.
   It is important to set the projection the algorithm uses
   (setProjectionEvaluator() function). If no projection is
   set, the planner will attempt to use the default projection
   associated to the state space. An exception is thrown if
   no default projection is available either.
   This implementation is intended for systems with differential constraints.
   @par External documentation
   I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
   in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
   [[PDF]](http://ioan.sucan.ro/files/pubs/wafr2008.pdf)
*/

/** \brief Kinodynamic Planning by Interior-Exterior Cell Exploration */
class KPIECELocal : public base::Planner {
public:

	/** \brief Constructor */
	KPIECELocal(const SpaceInformationPtr &si) : base::Planner(si, "KPIECELocal") {
		specs_.approximateSolutions = true;

		siC_ = si.get();
		nCloseSamples_ = 30;
		goalBias_ = 0.05;
		selectBorderFraction_ = 0.8;
		badScoreFactor_ = 0.45;
		goodScoreFactor_ = 0.9;
		tree_.grid.onCellUpdate(computeImportance, NULL);
		lastGoalMotion_ = NULL;

		Planner::declareParam<double>("goal_bias", this, &KPIECELocal::setGoalBias, &KPIECELocal::getGoalBias, "0.:.05:1.");
		Planner::declareParam<double>("border_fraction", this, &KPIECELocal::setBorderFraction, &KPIECELocal::getBorderFraction, "0.:0.05:1.");
		Planner::declareParam<unsigned int>("max_close_samples", this, &KPIECELocal::setMaxCloseSamplesCount, &KPIECELocal::getMaxCloseSamplesCount);
		Planner::declareParam<double>("bad_score_factor", this, &KPIECELocal::setBadCellScoreFactor, &KPIECELocal::getBadCellScoreFactor);
		Planner::declareParam<double>("good_score_factor", this, &KPIECELocal::setGoodCellScoreFactor, &KPIECELocal::getGoodCellScoreFactor);
	}

	virtual ~KPIECELocal() {
		freeMemory();
	}

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
		checkValidity();
		base::Goal *goal = pdef_->getGoal().get();

		while(const base::State *st = pis_.nextStart()) {
			Motion *motion = new Motion(siC_);
			si_->copyState(motion->state, st);
			siC_->nullControl(motion->control);
			addMotion(motion, 1.0);
		}

		if(tree_.grid.size() == 0) {
			OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		if(!controlSampler_)
			controlSampler_ = siC_->allocControlSampler();

		OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size);

		Motion *solution  = NULL;
		Motion *approxsol = NULL;
		double  approxdif = std::numeric_limits<double>::infinity();

		Control *rctrl = siC_->allocControl();

		std::vector<base::State *> states(siC_->getMaxControlDuration() + 1);
		std::vector<Grid::Coord>  coords(states.size());
		std::vector<Grid::Cell *>  cells(coords.size());

		for(unsigned int i = 0 ; i < states.size() ; ++i)
			states[i] = si_->allocState();

		// samples that were found to be the best, so far
		CloseSamples closeSamples(nCloseSamples_);

		while(ptc == false) {
			tree_.iteration++;

			/* Decide on a state to expand from */
			Motion     *existing = NULL;
			Grid::Cell *ecell = NULL;

			if(closeSamples.canSample() && rng_.uniform01() < goalBias_) {
				if(!closeSamples.selectMotion(existing, ecell))
					selectMotion(existing, ecell);
			} else
				selectMotion(existing, ecell);
			assert(existing);

			/* sample a random control */
			controlSampler_->sampleNext(rctrl, existing->control, existing->state);

			/* propagate */
			unsigned int cd = controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
			cd = siC_->propagateWhileValid(existing->state, rctrl, cd, states, false);

			/* if we have enough steps */
			if(cd >= siC_->getMinControlDuration()) {
				std::size_t avgCov_two_thirds = (2 * tree_.size) / (3 * tree_.grid.size());
				bool interestingMotion = false;
#ifdef STREAM_GRAPHICS
				streamPoint(existing->state, 1, 0, 0, 1);
#endif

				// split the motion into smaller ones, so we do not cross cell boundaries
				for(unsigned int i = 0 ; i < cd ; ++i) {
					projectionEvaluator_->computeCoordinates(states[i], coords[i]);
					cells[i] = tree_.grid.getCell(coords[i]);
					if(!cells[i])
						interestingMotion = true;
					else {
						if(!interestingMotion && cells[i]->data->motions.size() <= avgCov_two_thirds)
							interestingMotion = true;
					}
				}

				if(interestingMotion || rng_.uniform01() < 0.05) {
					unsigned int index = 0;
					while(index < cd) {
						unsigned int nextIndex = findNextMotion(coords, index, cd);
						Motion *motion = new Motion(siC_);
						si_->copyState(motion->state, states[nextIndex]);
						siC_->copyControl(motion->control, rctrl);
						motion->steps = nextIndex - index + 1;
						motion->parent = existing;

#ifdef STREAM_GRAPHICS
						streamPoint(motion->state, 1, 0, 0, 1);
#endif

						double dist = 0.0;
						bool solv = goal->isSatisfied(motion->state, &dist);
						Grid::Cell *toCell = addMotion(motion, dist);

						if(solv) {
							approxdif = dist;
							solution = motion;
							break;
						}
						if(dist < approxdif) {
							approxdif = dist;
							approxsol = motion;
						}

						closeSamples.consider(toCell, motion, dist);

						// new parent will be the newly created motion
						existing = motion;
						index = nextIndex + 1;
					}

					if(solution)
						break;
				}

				// update cell score
				ecell->data->score *= goodScoreFactor_;
			} else
				ecell->data->score *= badScoreFactor_;

			tree_.grid.update(ecell);
		}

		bool solved = false;
		bool approximate = false;
		if(solution == NULL) {
			solution = approxsol;
			approximate = true;
		}

		if(solution != NULL) {
			lastGoalMotion_ = solution;

			/* construct the solution path */
			std::vector<Motion *> mpath;
			while(solution != NULL) {
				mpath.push_back(solution);
				solution = solution->parent;
			}

			/* set the solution path */
			PathControl *path = new PathControl(si_);
			for(int i = mpath.size() - 1 ; i >= 0 ; --i)
				if(mpath[i]->parent)
					path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
				else
					path->append(mpath[i]->state);

			pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
			solved = true;
		}

		siC_->freeControl(rctrl);
		for(unsigned int i = 0 ; i < states.size() ; ++i)
			si_->freeState(states[i]);

		OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)",
		            getName().c_str(), tree_.size, tree_.grid.size(),
		            tree_.grid.countInternal(), tree_.grid.countExternal());

		return base::PlannerStatus(solved, approximate);
	}

	virtual void clear() {
		Planner::clear();
		controlSampler_.reset();
		freeMemory();
		tree_.grid.clear();
		tree_.size = 0;
		tree_.iteration = 1;
		lastGoalMotion_ = NULL;
	}

	/** In the process of randomly selecting states in the state
	    space to attempt to go towards, the algorithm may in fact
	    choose the actual goal state, if it knows it, with some
	    probability. This probability is a real number between 0.0
	    and 1.0; its value should usually be around 0.05 and
	    should not be too large. It is probably a good idea to use
	    the default value. */
	void setGoalBias(double goalBias) {
		goalBias_ = goalBias;
	}

	/** Get the goal bias the planner is using */
	double getGoalBias() const {
		return goalBias_;
	}

	/** \brief Set the fraction of time for focusing on the
	    border (between 0 and 1). This is the minimum fraction
	    used to select cells that are exterior (minimum
	    because if 95% of cells are on the border, they will
	    be selected with 95% chance, even if this fraction is
	    set to 90%)*/
	void setBorderFraction(double bp) {
		selectBorderFraction_ = bp;
	}

	/** \brief Get the fraction of time to focus exploration
	    on boundary */
	double getBorderFraction() const {
		return selectBorderFraction_;
	}

	/** \brief When extending a motion from a cell, the
	    extension can be successful or it can fail.  If the
	    extension is successful, the score of the cell is
	    multiplied by \e good. If the extension fails, the
	    score of the cell is multiplied by \e bad. These
	    numbers should be in the range (0, 1]. */
	void setCellScoreFactor(double good, double bad) {
		setGoodCellScoreFactor(good);
		setBadCellScoreFactor(bad);
	}

	/** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell fails */
	void setBadCellScoreFactor(double bad) {
		badScoreFactor_ = bad;
	}

	/** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell succeedes */
	void setGoodCellScoreFactor(double good) {
		goodScoreFactor_ = good;
	}

	/** \brief Get the factor that is multiplied to a cell's
	    score if extending a motion from that cell succeeded. */
	double getGoodCellScoreFactor() const {
		return goodScoreFactor_;
	}

	/** \brief Get the factor that is multiplied to a cell's
	    score if extending a motion from that cell failed. */
	double getBadCellScoreFactor() const {
		return badScoreFactor_;
	}

	/** \brief When motions reach close to the goal, they are stored in a separate queue
	    to allow biasing towards the goal. This function sets the maximum size of that queue. */
	void setMaxCloseSamplesCount(unsigned int nCloseSamples) {
		nCloseSamples_ = nCloseSamples;
	}

	/** \brief Get the maximum number of samples to store in the queue of samples that are close to the goal */
	unsigned int getMaxCloseSamplesCount() const {
		return nCloseSamples_;
	}

	/** \brief Set the projection evaluator. This class is
	    able to compute the projection of a given state. */
	void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator) {
		projectionEvaluator_ = projectionEvaluator;
	}

	/** \brief Set the projection evaluator (select one from
	    the ones registered with the state space). */
	void setProjectionEvaluator(const std::string &name) {
		projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
	}

	/** \brief Get the projection evaluator */
	const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const {
		return projectionEvaluator_;
	}

	virtual void setup() {
		Planner::setup();
		tools::SelfConfig sc(si_, getName());
		sc.configureProjectionEvaluator(projectionEvaluator_);

		if(badScoreFactor_ < std::numeric_limits<double>::epsilon() || badScoreFactor_ > 1.0)
			throw Exception("Bad cell score factor must be in the range (0,1]");
		if(goodScoreFactor_ < std::numeric_limits<double>::epsilon() || goodScoreFactor_ > 1.0)
			throw Exception("Good cell score factor must be in the range (0,1]");
		if(selectBorderFraction_ < std::numeric_limits<double>::epsilon() || selectBorderFraction_ > 1.0)
			throw Exception("The fraction of time spent selecting border cells must be in the range (0,1]");

		tree_.grid.setDimension(projectionEvaluator_->getDimension());
	}

	virtual void getPlannerData(base::PlannerData &data) const {
		Planner::getPlannerData(data);

		Grid::CellArray cells;
		tree_.grid.getCells(cells);

		double delta = siC_->getPropagationStepSize();

		if(lastGoalMotion_)
			data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

		for(unsigned int i = 0 ; i < cells.size() ; ++i) {
			for(unsigned int j = 0 ; j < cells[i]->data->motions.size() ; ++j) {
				const Motion *m = cells[i]->data->motions[j];
				if(m->parent) {
					if(data.hasControls())
						data.addEdge(base::PlannerDataVertex(m->parent->state),
						             base::PlannerDataVertex(m->state, cells[i]->border ? 2 : 1),
						             control::PlannerDataEdgeControl(m->control, m->steps * delta));
					else
						data.addEdge(base::PlannerDataVertex(m->parent->state),
						             base::PlannerDataVertex(m->state, cells[i]->border ? 2 : 1));
				} else
					data.addStartVertex(base::PlannerDataVertex(m->state, cells[i]->border ? 2 : 1));

				// A state created as a parent first may have an improper tag variable
				data.tagState(m->state, cells[i]->border ? 2 : 1);
			}
		}
	}

protected:

	/** \brief Representation of a motion for this algorithm */
	struct Motion {
		Motion() : state(NULL), control(NULL), steps(0), parent(NULL) {
		}

		/** \brief Constructor that allocates memory for the state and the control */
		Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL) {
		}

		~Motion() {
		}

		/** \brief The state contained by this motion */
		base::State       *state;

		/** \brief The control contained by this motion */
		Control           *control;

		/** \brief The number of steps the control is applied for */
		unsigned int       steps;

		/** \brief The parent motion in the exploration tree */
		Motion            *parent;
	};

	/** \brief The data held by a cell in the grid of motions */
	struct CellData {
		CellData() : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0) {
		}

		~CellData() {
		}

		/** \brief The set of motions contained in this grid cell */
		std::vector<Motion *> motions;

		/** \brief A measure of coverage for this cell. For
		    this implementation, this is the sum of motion
		    durations */
		double               coverage;

		/** \brief The number of times this cell has been
		    selected for expansion */
		unsigned int         selections;

		/** \brief A heuristic score computed based on
		    distance to goal (if available), successes and
		    failures at expanding from this cell. */
		double               score;

		/** \brief The iteration at which this cell was created */
		unsigned int         iteration;

		/** \brief The computed importance (based on other class members) */
		double               importance;
	};

	/** \brief Definintion of an operator passed to the Grid
	    structure, to order cells by importance */
	struct OrderCellsByImportance {
		bool operator()(const CellData *const a, const CellData *const b) const {
			return a->importance > b->importance;
		}
	};

	/** \brief The datatype for the maintained grid datastructure */
	typedef GridB<CellData *, OrderCellsByImportance> Grid;

	/** \brief Information about a known good sample (closer to the goal than others) */
	struct CloseSample {
		/** \brief Constructor fully initializes the content of this structure */
		CloseSample(Grid::Cell *c, Motion *m, double d) : cell(c), motion(m), distance(d) {
		}

		/** \brief The cell of the motion that is close to the goal */
		Grid::Cell *cell;

		/** \brief The motion that is close to the goal */
		Motion     *motion;

		/** \brief The distance to the goal. This value is increased over time, as the number of selections for this sample increases */
		double      distance;

		/** \brief Sort samples in accordance to their distance to the goal */
		bool operator<(const CloseSample &other) const {
			return distance < other.distance;
		}
	};

	/** \brief Bounded set of good samples */
	struct CloseSamples {
		/** \brief Construct an object to maintain a set of at most \e size samples */
		CloseSamples(unsigned int size) : maxSize(size) {
		}

		/** \brief Evaluate whether motion \e motion, part of
		    cell \e cell is good enough to be part of the set
		    of samples closest to the goal, given its distance
		    to the goal is \e distance. If so, add it to the
		    set and return true. Otherwise, return false.*/
		bool consider(Grid::Cell *cell, Motion *motion, double distance) {
			if(samples.empty()) {
				CloseSample cs(cell, motion, distance);
				samples.insert(cs);
				return true;
			}
			// if the sample we're considering is closer to the goal than the worst sample in the
			// set of close samples, we include it
			if(samples.rbegin()->distance > distance) {
				// if the inclusion would go above the maximum allowed size,
				// remove the last element
				if(samples.size() >= maxSize)
					samples.erase(--samples.end());
				CloseSample cs(cell, motion, distance);
				samples.insert(cs);
				return true;
			}

			return false;
		}

		/** \brief Select the top sample (closest to the goal)
		    and update its position in the set subsequently
		    (pretend the distance to the goal is
		    larger). Returns true if the sample selection is
		    successful. */
		bool selectMotion(Motion *&smotion, Grid::Cell *&scell) {
			if(samples.size() > 0) {
				scell = samples.begin()->cell;
				smotion = samples.begin()->motion;
				// average the highest & lowest distances and multiply by CLOSE_MOTION_DISTANCE_INFLATION_FACTOR
				// (make the distance appear artificially longer)
				double d = (samples.begin()->distance + samples.rbegin()->distance) * (CLOSE_MOTION_DISTANCE_INFLATION_FACTOR / 2.0);
				samples.erase(samples.begin());
				consider(scell, smotion, d);
				return true;
			}
			return false;
		}

		/** \brief Return true if samples can be selected from this set */
		bool canSample() const {
			return samples.size() > 0;
		}

		/** \brief Maximum number of samples to maintain */
		unsigned int          maxSize;

		/** \brief The maintained samples */
		std::set<CloseSample> samples;
	};


	/** \brief The data defining a tree of motions for this algorithm */
	struct TreeData {
		TreeData() : grid(0), size(0), iteration(1) {
		}

		/** \brief A grid containing motions, imposed on a
		    projection of the state space */
		Grid         grid;

		/** \brief The total number of motions (there can be
		    multiple per cell) in the grid */
		unsigned int size;

		/** \brief The number of iterations performed on this tree */
		unsigned int iteration;
	};

	/** \brief This function is provided as a calback to the
	    grid datastructure to update the importance of a
	    cell */
	static void computeImportance(Grid::Cell *cell, void *) {
		CellData &cd = *(cell->data);
		cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
	}

	/** \brief Free all the memory allocated by this planner */
	void freeMemory() {
		freeGridMotions(tree_.grid);
	}

	/** \brief Free the memory for the motions contained in a grid */
	void freeGridMotions(Grid &grid) {
		for(Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
			freeCellData(it->second->data);
	}

	/** \brief Free the memory for the data contained in a grid cell */
	void freeCellData(CellData *cdata) {
		for(unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
			freeMotion(cdata->motions[i]);
		delete cdata;
	}

	/** \brief Free the memory for a motion */
	void freeMotion(Motion *motion) {
		if(motion->state)
			si_->freeState(motion->state);
		if(motion->control)
			siC_->freeControl(motion->control);
		delete motion;
	}

	/** \brief Add a motion to the grid containing motions. As
	    a hint, \e dist specifies the distance to the goal
	    from the state of the motion being added. The function
	    Returns the number of cells created to accommodate the
	    new motion (0 or 1). */
	Grid::Cell *addMotion(Motion *motion, double dist) {
		Grid::Coord coord;
		projectionEvaluator_->computeCoordinates(motion->state, coord);
		Grid::Cell *cell = tree_.grid.getCell(coord);
		if(cell) {
			cell->data->motions.push_back(motion);
			cell->data->coverage += motion->steps;
			tree_.grid.update(cell);
		} else {
			cell = tree_.grid.createCell(coord);
			cell->data = new CellData();
			cell->data->motions.push_back(motion);
			cell->data->coverage = motion->steps;
			cell->data->iteration = tree_.iteration;
			cell->data->selections = 1;
			cell->data->score = (1.0 + log((double)(tree_.iteration))) / (DISTANCE_TO_GOAL_OFFSET + dist);
			tree_.grid.add(cell);
		}
		tree_.size++;
		return cell;
	}

	/** \brief Select a motion and the cell it is part of from
	    the grid of motions. This is where preference is given
	    to cells on the boundary of the grid.*/
	bool selectMotion(Motion *&smotion, Grid::Cell *&scell) {
		scell = rng_.uniform01() < std::max(selectBorderFraction_, tree_.grid.fracExternal()) ?
		        tree_.grid.topExternal() : tree_.grid.topInternal();

		// We are running on finite precision, so our update scheme will end up
		// with 0 values for the score. This is where we fix the problem
		if(scell->data->score < std::numeric_limits<double>::epsilon()) {
			OMPL_DEBUG("%s: Numerical precision limit reached. Resetting costs.", getName().c_str());
			std::vector<CellData *> content;
			content.reserve(tree_.grid.size());
			tree_.grid.getContent(content);
			for(std::vector<CellData *>::iterator it = content.begin() ; it != content.end() ; ++it)
				(*it)->score += 1.0 + log((double)((*it)->iteration));
			tree_.grid.updateAll();
		}

		if(scell && !scell->data->motions.empty()) {
			scell->data->selections++;
			smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
			return true;
		} else
			return false;
	}

	/** \brief When generated motions are to be added to the tree of motions, they often need to be split, so they don't cross cell boundaries.
	    Given that a motion starts out in the cell \e origin and it crosses the cells in \e coords[\e index] through \e coords[\e last] (inclusively),
	    return the index of the state to be used in the next part of the motion (that is within a cell). This will be a value between \e index and \e last. */
	unsigned int findNextMotion(const std::vector<Grid::Coord> &coords, unsigned int index, unsigned int count) {
		for(unsigned int i = index + 1 ; i < count ; ++i)
			if(coords[i] != coords[index])
				return i - 1;

		return count - 1;
	}

	/** \brief A control sampler */
	ControlSamplerPtr             controlSampler_;

	/** \brief The tree datastructure */
	TreeData                      tree_;

	/** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
	const SpaceInformation       *siC_;

	/** \brief This algorithm uses a discretization (a grid)
	    to guide the exploration. The exploration is imposed
	    on a projection of the state space. */
	base::ProjectionEvaluatorPtr  projectionEvaluator_;

	/** \brief When extending a motion from a cell, the
	    extension can be successful. If it is, the score of the
	    cell is multiplied by this factor. */
	double                        goodScoreFactor_;

	/** \brief When extending a motion from a cell, the
	    extension can fail. If it is, the score of the cell is
	    multiplied by this factor. */
	double                        badScoreFactor_;

	/** \brief When motions reach close to the goal, they are stored in a separate queue
	    to allow biasing towards the goal. This variable specifies the maximum number of samples
	    to keep in that queue. */
	unsigned int                  nCloseSamples_;

	/** \brief The fraction of time to focus exploration on
	    the border of the grid. */
	double                        selectBorderFraction_;

	/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
	double                        goalBias_;

	/** \brief The random number generator */
	RNG                           rng_;

	/** \brief The most recent goal motion.  Used for PlannerData computation */
	Motion                       *lastGoalMotion_;
};

}
}


#endif