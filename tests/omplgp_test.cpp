/**
 * OMPL global planner tests.
 * Author: Abdul Rahman Dabbour
 * Email: ardabbour@gmail.com
 * License: MIT
 */

#include <gtest/gtest.h>
#include <omplgp/omplgp.hpp>

TEST(BasicRRTTest, BasicRRTTest) {
  // clang-format off
  omplgp::CostMap2D costmap{
      {0.2, 0.8, 0.3, 0.4, 0.3},
      {0.3, 0.6, 0.1, 0.5, 0.3},
      {0.1, 0.9, 0.4, 0.7, 0.1},
      {0.4, 0.7, 0.1, 0.8, 0.4},
      {0.2, 0.3, 0.2, 0.6, 0.2},
  };
  // clang-format on

  omplgp::Point2D start(0.5, 0.5);
  omplgp::Point2D goal(4.5, 4.5);
  omplgp::Plan2D plan;

  omplgp::OMPL2DPlanner rrt(omplgp::Algorithm::RRT, costmap, 0.5, 2.0);

  // 2 seconds is ample time to find a goal, even on an embedded system
  ASSERT_TRUE(rrt.plan(start, goal, plan));

  // ALl costs must be less than 0.5
  for (const auto &i : plan) {
    EXPECT_TRUE(costmap[(int)floor(i.x)][(int)floor(i.y)] < 0.5);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}