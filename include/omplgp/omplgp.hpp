/**
 * OMPL global planner prototype.
 * Author: Abdul Rahman Dabbour
 * Email: ardabbour@gmail.com
 * License: MIT
 */

#ifndef OMPLGP_H
#define OMPLGP_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

namespace omplgp {

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief algorithms which can be used to plan in. For reference:
 * https://ompl.kavrakilab.org/planners.html
 */
enum Algorithm {
  AUTO,

  PRM,
  PRMstar,
  LazyPRM,
  LazyPRMstar,

  RRT,
  RRTConnect,
  RRTstar,
  RRTsharp,
  LazyRRT,
  InformedRRTstar,

  KPIECE,
  BKPIECE,
  LBKPIECE
};

/**
 * @brief a point in 2D space
 */
struct Point2D {
  /**
   * @brief constructor to create a point at a custom point in 2D space
   * @param x the x coordinate of the point
   * @param y the y coordinate of the point
   */
  Point2D(const double x_coordinate = 0.0, const double y_coordinate = 0.0);

  /**
   * @brief the x coordinate of the point
   */
  double x;

  /**
   * @brief the y coordinate of the point
   */
  double y;
};

/**
 * @brief 2D costmap, where the indices indicate the location and the value
 * indicates the cost
 */
using CostMap2D = std::vector<std::vector<double>>;

/**
 * @brief a sequence of points in 2D space
 */
using Plan2D = std::vector<Point2D>;

/**
 * @brief the main planner class
 */
class OMPL2DPlanner {
public:
  /**
   * @brief constructor to create a global planner
   * @param algorithm the algorithm to use for planning
   * @param costmap the costmap to plan in
   * @param obstacle_cost the minimum cost of an obstacle
   * @param timeout the maximum amount of time the planner will run for
   */
  OMPL2DPlanner(const Algorithm algorithm = Algorithm::AUTO,
                const CostMap2D costmap = CostMap2D(),
                const double obstacle_cost = 0.5, const double timeout = 1.0);

  /**
   * @brief gets the algorithm
   */
  const Algorithm getAlgorithm();

  /**
   * @brief sets the algorithm
   * @param algorithm the algorithm to set
   */
  void setAlgorithm(const Algorithm algorithm);

  /**
   * @brief gets the timeout
   * @returns the timeout
   */
  const double getTimeout();

  /**
   * @brief sets the timeout
   * @param timeout the timeout to set
   */
  void setTimeout(const double timeout);

  /**
   * @brief gets the costmap
   * @returns the costmap
   */
  const CostMap2D getCostMap();

  /**
   * @brief sets the costmap
   * @param costmap the costmap to set
   */
  void setCostMap(const CostMap2D &costmap);

  /**
   * @brief gets the obstacle cost
   * @returns the obstacle cost
   */
  const double getObstacleCost();

  /**
   * @brief sets the obstacle cost
   * @param obstacle_cost the obstacle cost to set
   */
  void setObtacleCost(const double &obstacle_cost);

  /**
   * @brief plans for a path that connects two points
   * @param start the starting point
   * @param goal the goal point
   * @param plan the empty vector to fill the plan in
   * @returns if the planning was successful or not
   */
  const bool plan(const Point2D &start, const Point2D &goal, Plan2D &plan);

protected:
  /**
   * @brief the timeout of the planner
   */
  double timeout_;

  /**
   * @brief the costmap of the planner
   */
  CostMap2D costmap_;

  /**
   * @brief the algorithm of the planner
   */
  Algorithm algorithm_;

  /**
   * @brief the obstacle cost of the planner
   */
  double obstacle_cost_;

  /**
   * @brief the space of the planner
   */
  std::shared_ptr<ob::RealVectorStateSpace> space_;

  /**
   * @brief the simple setup pointer of the planner
   */
  og::SimpleSetupPtr ss_;

  /**
   * @brief checks if a state is valid, i.e. a point is not in occupied space
   * @param state the state pointer
   * @returns true if the state is not in occupied space, false otherwise
   */
  const bool isStateValid(const ob::State *state);

  /**
   * @brief modifies the space of the planner to account for new bounds
   */
  void setSpaceBounds();
};

} // namespace omplgp
#endif