# OMPL Global Planner

A global planner powered by [OMPL](http://ompl.kavrakilab.org/). This was made with the navigation stack of ROS in mind, but being completely independent of them both. ROS1 and ROS2 global planner plugins are planned (pun intended).

## Requirements

| Requirement | Domain | Version Tested |
| :---: | :---: | :---: |
| [OMPL](http://ompl.kavrakilab.org/) | Runtime | 1.4.2 |
| [Google Test](https://github.com/google/googletest) | Testing | 1.10.x |

## Installation

Use [CMake](https://cmake.org/) to build omplgp.

``` bash
cmake .. # use -DBUILD_TESTS as applicable
make
make install # or make package to create a debian to install
```

## Usage

``` C++
#include <omplgp/omplgp.hpp>

int main() {
  // Let's create a simple costmap
  omplgp::CostMap2D my_cmap{
      {0.2, 0.1, 0.3, 0.4, 0.3},
      {0.4, 0.1, 0.5, 0.3, 0.8},
      {0.6, 0.3, 0.7, 0.3, 0.1},
      {0.4, 0.2, 0.1, 0.9, 0.4},
      {0.3, 0.8, 0.2, 0.8, 0.2},
  };

  // Set anything >= 0.5 to be recognized as an obstacle
  double my_ocost(0.5);

  // Let's use a simple RRT planner
  omplgp::Algorithm my_algo(omplgp::Algorithm::RRT);

  // Set the maximum time to use for planning to 2 seconds
  double my_tout(2.0);

  // Set the two extremes of the costmap to be the start and goal points
  omplgp::Point2D my_start(0.5, 0.5);
  omplgp::Point2D my_goal(4.5, 4.5);

  // Create the planner
  omplgp::Planner2D my_rrt(my_algo, my_cmap, my_ocost, my_tout);

  // Find a plan!
  omplgp::Plan2D my_plan;
  bool success = my_rrt.plan(my_start, my_goal, my_plan);
  if (success) {
    std::cout << "Succeded in finding a plan! :)" << std::endl;
    std::cout << "Here is the plan!" << std::endl;
    for (const auto &i : my_plan) {
      std::cout << "x: " << i.x << std::endl;
      std::cout << "y: " << i.y << std::endl << std::endl;
    }
  } else {
    std::cout << "Failed in finding a plan! :(" << std::endl;
  }
}
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](LICENSE)