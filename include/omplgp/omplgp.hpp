/**
 * OMPL global planner prototype.
 * Author: Abdul Rahman Dabbour
 * Email: ardabbour@gmail.com
 * License: MIT
 */

#ifndef OMPLGP_H
#define OMPLGP_H

/**
 * Base includes
 */
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

/**
 * Control includes
 */
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

/**
 * Geometric includes
 */
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
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

namespace omplgp {

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

/**
 * @brief sampling-based algorithms for control problems. For reference:
 * https://ompl.kavrakilab.org/planners.html
 */
enum ControlAlgorithm {
  AUTO,
  EST,
  KPIECE,
  PDST,
  RRT,
  SST,
};

/**
 * @brief sampling-based algorithms for geometric problems. For reference:
 * https://ompl.kavrakilab.org/planners.html
 */
enum GeometricAlgorithm {
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
struct QuadrotorPose {
  /**
   * @brief constructor to create a kinematic state of a quadrotor
   * @param x the x position of the quadrotor
   * @param y the y position of the quadrotor
   * @param z the z position of the quadrotor
   * @param roll the roll orientation of the quadrotor
   * @param pitch the pitch orientation of the quadrotor
   * @param yaw the yaw orientation of the quadrotor
   */
  QuadrotorPose(const double &x = 0.0, const double &y = 0.0,
                const double &z = 0.0, const double &roll = 0.0,
                const double &pitch = 0.0, const double &yaw = 0.0);

  /**
   * @brief the x position of the quadrotor
   */
  double x;

  /**
   * @brief the y position of the quadrotor
   */
  double y;

  /**
   * @brief the z position of the quadrotor
   */
  double z;

  /**
   * @brief the roll orientation of the quadrotor
   */
  double roll;

  /**
   * @brief the pitch orientation of the quadrotor
   */
  double pitch;

  /**
   * @brief the yaw orientation of the quadrotor
   */
  double yaw;
};

/**
 * @brief Quadrotor kinematic control space
 */
class QuadrotorKinematicControlSpace : public oc::RealVectorControlSpace {
public:
  explicit QuadrotorKinematicControlSpace(const ob::StateSpacePtr &state_space)
      : oc::RealVectorControlSpace(state_space, 6) {}
};

/**
 * @brief Quadrotor dynamic control space
 */
class QuadrotorDynamicControlSpace : public oc::RealVectorControlSpace {
public:
  explicit QuadrotorDynamicControlSpace(const ob::StateSpacePtr &state_space)
      : oc::RealVectorControlSpace(state_space, 6) {}
};

/**
 * @brief 3D costmap, where the indices indicate the location and the value
 * indicates the cost
 */
using CostMap3D = std::vector<std::vector<std::vector<double>>>;

/**
 * @brief a sequence of points in 3D space
 */
using Plan3D = std::vector<QuadrotorPose>;

/**
 * @brief the main planner class
 */
template <typename T> class OMPL3DPlanner {
public:
  /**
   * @brief constructor to create a global planner
   * @param algorithm the algorithm to use for planning
   * @param costmap the costmap to plan in
   * @param obstacle_cost the minimum cost of an obstacle
   * @param timeout the maximum amount of time the planner will run for
   */
  OMPL3DPlanner(const T &algorithm = T::AUTO,
                const CostMap3D &costmap = CostMap3D(),
                const double &obstacle_cost = 0.5, const double &timeout = 1.0);

  /**
   * @brief gets the algorithm
   */
  const T getAlgorithm();

  /**
   * @brief sets the algorithm
   * @param algorithm the algorithm to set
   */
  void setAlgorithm(const T &algorithm);

  /**
   * @brief gets the timeout
   * @returns the timeout
   */
  const double getTimeout();

  /**
   * @brief sets the timeout
   * @param timeout the timeout to set
   */
  void setTimeout(const double &timeout);

  /**
   * @brief gets the costmap
   * @returns the costmap
   */
  const CostMap3D getCostMap();

  /**
   * @brief sets the costmap
   * @param costmap the costmap to set
   */
  void setCostMap(const CostMap3D &costmap);

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
  const bool plan(const QuadrotorPose &start, const QuadrotorPose &goal,
                  Plan3D &plan);

protected:
  /**
   * @brief the timeout of the planner
   */
  double timeout_;

  /**
   * @brief the costmap of the planner
   */
  CostMap3D costmap_;

  /**
   * @brief the algorithm of the planner
   */
  T algorithm_;

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