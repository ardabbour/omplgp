/**
 * OMPL global planner implementation.
 * Author: Abdul Rahman Dabbour
 * Email: ardabbour@gmail.com
 * License: MIT
 */

#include <omplgp/omplgp.hpp>

namespace omplgp {

Point2D::Point2D(const double x_coordinate, const double y_coordinate) {
  x = x_coordinate;
  y = y_coordinate;
}

Planner2D::Planner2D(const Algorithm algorithm, const CostMap2D costmap,
                     const double obstacle_cost, double timeout)
    : costmap_(costmap), obstacle_cost_(obstacle_cost), timeout_(timeout),
      space_(std::make_shared<ob::RealVectorStateSpace>()) {
  space_->addDimension();
  space_->addDimension();
  setSpaceBounds();

  ss_ = std::make_shared<og::SimpleSetup>(space_);
  ss_->setStateValidityChecker(
      [this](const ob::State *state) { return isStateValid(state); });
  space_->setup();

  setAlgorithm(algorithm);
}

const Algorithm Planner2D::getAlgorithm() { return algorithm_; }
void Planner2D::setAlgorithm(const Algorithm algorithm) {
  switch (algorithm) {

  case AUTO:
    ss_->setPlanner(ob::PlannerPtr());
    break;

  case PRM:
    ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
    break;
  case PRMstar:
    ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
    break;
  case LazyPRM:
    ss_->setPlanner(std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
    break;
  case LazyPRMstar:
    ss_->setPlanner(
        std::make_shared<og::LazyPRMstar>(ss_->getSpaceInformation()));
    break;

  case RRT:
    ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
    break;
  case RRTConnect:
    ss_->setPlanner(
        std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
    break;
  case RRTstar:
    ss_->setPlanner(std::make_shared<og::RRTstar>(ss_->getSpaceInformation()));
    break;
  case RRTsharp:
    ss_->setPlanner(std::make_shared<og::RRTsharp>(ss_->getSpaceInformation()));
    break;
  case LazyRRT:
    ss_->setPlanner(std::make_shared<og::LazyRRT>(ss_->getSpaceInformation()));
    break;
  case InformedRRTstar:
    ss_->setPlanner(
        std::make_shared<og::InformedRRTstar>(ss_->getSpaceInformation()));
    break;

  case KPIECE:
    ss_->setPlanner(std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
    break;
  case BKPIECE:
    ss_->setPlanner(std::make_shared<og::BKPIECE1>(ss_->getSpaceInformation()));
    break;
  case LBKPIECE:
    ss_->setPlanner(
        std::make_shared<og::LBKPIECE1>(ss_->getSpaceInformation()));
    break;

  default:
    break;
  }
  algorithm_ = algorithm;
}

const double Planner2D::getTimeout() { return timeout_; }
void Planner2D::setTimeout(const double timeout) { timeout_ = timeout; }

const CostMap2D Planner2D::getCostMap() { return costmap_; }
void Planner2D::setCostMap(const CostMap2D &costmap) {
  costmap_ = costmap;
  setSpaceBounds();
}

const double Planner2D::getObstacleCost() { return obstacle_cost_; }
void Planner2D::setObtacleCost(const double &obstacle_cost) {
  obstacle_cost_ = obstacle_cost;
}

const bool Planner2D::plan(const Point2D &start, const Point2D &goal,
                           Plan2D &plan) {
  if (!ss_) {
    std::cerr << "No solution found" << std::endl;
    return false;
  }

  ob::ScopedState<ob::RealVectorStateSpace> start_state(ss_->getStateSpace());
  start_state[0] = start.x;
  start_state[1] = start.y;

  ob::ScopedState<ob::RealVectorStateSpace> goal_state(ss_->getStateSpace());
  goal_state[0] = goal.x;
  goal_state[1] = goal.y;

  ss_->setStartAndGoalStates(start_state, goal_state);

  ob::PlannerStatus solved = ss_->solve(timeout_);
  if (solved) {
    for (const auto &i : ss_->getSolutionPath().getStates()) {
      Point2D point;
      point.x = (double)i->as<ob::RealVectorStateSpace::StateType>()->values[0];
      point.y = (double)i->as<ob::RealVectorStateSpace::StateType>()->values[1];
      plan.push_back(point);
    }
  } else {
    std::cerr << "No solution found" << std::endl;
    return false;
  }
  return true;
}

void Planner2D::setSpaceBounds() {
  const double min_x = 0;
  const double max_x = costmap_.size();
  const double min_y = 0;
  const double max_y = costmap_[0].size();

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, min_x);
  bounds.setHigh(0, max_x);
  bounds.setLow(1, min_y);
  bounds.setHigh(1, max_y);
  space_->setBounds(bounds);
}

const bool Planner2D::isStateValid(const ob::State *state) {
  const int x =
      (int)floor(state->as<ob::RealVectorStateSpace::StateType>()->values[0]);
  const int y =
      (int)floor(state->as<ob::RealVectorStateSpace::StateType>()->values[1]);

  if (costmap_[x][y] >= obstacle_cost_) {
    return false;
  }
  return true;
}
} // namespace omplgp
