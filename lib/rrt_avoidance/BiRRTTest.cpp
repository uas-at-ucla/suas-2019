#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include "getopt.h"

#include "2dplane/2dplane.hpp"
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "planning/Path.hpp"

#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;

namespace RRT {

bool plot = false;

TEST(BiRRT, Instantiation) {
  BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50), hash,
                        dimensions);
}

TEST(BiRRT, getPath) {
  Vector2d start = {1, 1}, goal = {50, 50};

  std::shared_ptr<RRT::GridStateSpace> state_space =
      make_shared<GridStateSpace>(50, 50, 50, 50);

  vector<double> obs_x, obs_y;
  for (int x = 10; x < 30; x++) {
    for (int y = 10; y < 30; y++) {
      obs_x.push_back(x);
      obs_y.push_back(y);
      state_space->obstacleGrid().obstacleAt(Vector2i(x, y)) = true;
    }
  }

  for (int x = 10; x < 50; x++) {
    for (int y = 30; y < 45; y++) {
      obs_x.push_back(x);
      obs_y.push_back(y);
      state_space->obstacleGrid().obstacleAt(Vector2i(x, y)) = true;
    }
  }

  //  toggle the obstacle state of clicked square

  BiRRT<Vector2d> biRRT(state_space, hash, dimensions);
  biRRT.setStartState(start);
  biRRT.setGoalState(goal);
  biRRT.setStepSize(1);
  biRRT.setMaxIterations(10000);

  bool success = biRRT.run();
  ASSERT_TRUE(success);

  vector<Vector2d> path = biRRT.getPath();
  vector<Vector2d> smooth_path(path);
  RRT::SmoothPath<Vector2d>(smooth_path, *state_space);

  vector<double> x, y;
  for (Vector2d node : path) {
    x.push_back(node(0));
    y.push_back(node(1));
  }

  vector<double> smooth_x, smooth_y;
  for (Vector2d node : smooth_path) {
    smooth_x.push_back(node(0));
    smooth_y.push_back(node(1));
  }

  if(plot) {
  ::matplotlibcpp::xkcd();
  ::matplotlibcpp::plot(x, y);
  ::matplotlibcpp::plot(smooth_x, smooth_y);
  ::matplotlibcpp::plot(obs_x, obs_y);
  ::matplotlibcpp::xlim(0, 50);
  ::matplotlibcpp::ylim(0, 50);
  ::matplotlibcpp::show();
  }

  // path should contain at least two points (start and end)
  ASSERT_GE(path.size(), 2);

  // The given start and goal points should be the first and last points of
  // the path, respectively.
  EXPECT_EQ(start, path.front());
  EXPECT_EQ(goal, path.back());
}

}  // namespace RRT

int main(int argc, char **argv) {
  static struct option getopt_options[] = {
    {"plot", no_argument, 0, 'p'},
    {0, 0, 0, 0}
  };

  while(1) {
    int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
    if(opt == -1) break;

    switch(opt) {
      case 'p':
        ::RRT::plot = true;
        break;
      default:
        exit(1);
        break;
    }
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
