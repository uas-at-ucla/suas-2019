#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include "getopt.h"

#include <math.h>
#include <stdio.h>
#include <cmath>
#include "2dplane/2dplane.hpp"
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "lib/physics_structs/physics_structs.h"
#include "matplotlibcpp.h"
#include "planning/Path.hpp"

const int DISPLAY_MODE = 1;

using namespace std;
using namespace Eigen;

namespace RRT {

bool plot = false;

TEST(BiRRT, Instantiation) {
  BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(100, 100, 100, 100), hash,
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

  if (plot) {
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

TEST(BiRRT, GeocoordToObstacleGridTest) {
  int number_of_obstacles = 2;
  int rows = 2 + number_of_obstacles;

  ::Eigen::MatrixXd m(rows, 3);

  ::std::cout.precision(10);

  // Load initial start/end and obstacle values.
  m(0, 0) = 3e-3;
  m(0, 1) = 3e-3;
  m(0, 2) = 0;
  m(1, 0) = 1e-3;
  m(1, 1) = 1e-3;
  m(1, 2) = 0;
  for(int i = 2;i < rows;i++) {
    m(i, 0) = 2e-3;
    m(i, 1) = 1.6e-3 + (i - 2) * 1.2e-3;
    m(i, 2) = 50;
  }

  // Shift everything so that the drone position at the origin.
  ::Eigen::MatrixXd m_trans(rows, 3);
  m_trans = m.row(0).replicate(rows, 1);
  m -= m_trans;

  // Make the start->end points exist in the first quadrant.
  ::Eigen::MatrixXd reflection_m = MatrixXd::Identity(3, 3);
  reflection_m(0, 0) = (m(1, 0) >= 0) ? 1 : -1;
  reflection_m(1, 1) = (m(1, 1) >= 0) ? 1 : -1;
  m *= reflection_m;

  // Convert coordinate components to meters.
  for (int i = 0; i < rows; i++) {
    m(i, 0) = ::spinny::GetDistance2D({0, 0, 0}, {m(i, 0), 0, 0});
    m(i, 1) = ::spinny::GetDistance2D({0, 0, 0}, {0, m(i, 1), 0});
  }

  ::std::cout << m << ::std::endl;

  // Scale everything to be within the 0 -> 50 obstacle grid used by RRT.
  const int kDimension = 50;
  double meter_width = m(1, 0);
  double meter_height = m(1, 1);
  double longest_dim = meter_width > meter_height ? meter_width : meter_height;

  double scale = (kDimension - 2) / longest_dim;
  m *= scale;

  // Leave a 1 unit buffer around everything to make rrt more stable.
  ::Eigen::MatrixXd m_shift(rows, 3);
  ::Eigen::MatrixXd m_shift_row(1, 3);
  m_shift_row << 1, 1, 0;
  m_shift = m_shift_row.replicate(rows, 1);
  m += m_shift;
  ::std::cout << m << ::std::endl;

  // Run the RRT.
  std::shared_ptr<RRT::GridStateSpace> state_space =
      make_shared<GridStateSpace>(kDimension, kDimension, kDimension,
                                  kDimension);

  Vector2d start = {m(0, 0) + 1, m(0, 1) + 1};
  Vector2d goal = {m(1, 0) - 1, m(1, 1) - 1};

  BiRRT<Vector2d> biRRT(state_space, hash, dimensions);
  vector<double> obs_x, obs_y;

  for(int i = 2;i < rows;i++) {
    int obstacle_radius = m(i, 2);
    for (int offset_x = -obstacle_radius; offset_x <= obstacle_radius;
         offset_x++) {
      int y_max = sqrt(pow(obstacle_radius, 2) - pow(offset_x, 2)) + 1;

      for (int offset_y = -y_max; offset_y <= y_max; offset_y++) {
        int x = m(i, 0) + offset_x;
        int y = m(i, 1) + offset_y;
        if(x < 0 || x >= 50 | y < 0 || y >= 50) {
          continue;
        }

        state_space->obstacleGrid().obstacleAt(Vector2i(x, y)) = true;

        obs_x.push_back(x);
        obs_y.push_back(y);
      }
    }
  }

  biRRT.setStartState(start);
  biRRT.setGoalState(goal);
  biRRT.setStepSize(1);
  biRRT.setMaxIterations(10000);
  bool success = biRRT.run();
  vector<Vector2d> path = biRRT.getPath();
  vector<Vector2d> smooth_path(path);
  RRT::SmoothPath<Vector2d>(smooth_path, *state_space);

  ::Eigen::MatrixXd rrt_out_m(smooth_path.size(), 3);

  vector<double> smooth_x, smooth_y;
  for (int i = 0; i < smooth_path.size(); i++) {
    rrt_out_m(i, 0) = smooth_path[i][0];
    rrt_out_m(i, 1) = smooth_path[i][1];
    rrt_out_m(i, 2) = 0;

    smooth_x.push_back(smooth_path[i][0]);
    smooth_y.push_back(smooth_path[i][1]);
  }

  cout << rrt_out_m << endl;

  ::Eigen::MatrixXd meter_out_m(smooth_path.size(), 3);
  meter_out_m = (1 / scale) * rrt_out_m;

  // TODO(akaash): Only add the buffer after we've gotten the whole thing to
  // work in the first place.
  /*
  // Add a buffer to take into account obstacles that are not directly in the
  // frame of the start and end points.
  double dist = sqrt((meter_pos_m(1, 0) - meter_pos_m(0, 0)) *
                         (meter_pos_m(1, 0) - meter_pos_m(0, 0)) +
                     (meter_pos_m(1, 1) - meter_pos_m(0, 1)) *
                         (meter_pos_m(1, 1) - meter_pos_m(0, 1)));

  double buffer = dist * 0.2;
  ::Eigen::MatrixXd buffer_m(rows, 3);
  for (int i = 0; i < rows; i++) {
    buffer_m(i, 0) = buffer;
    buffer_m(i, 1) = buffer;
    buffer_m(i, 2) = 0;
  }
  meter_pos_m += buffer_m;
  */
  // TODO(akaash): Do buffer stuff later.
  /*
  ::Eigen::MatrixXd buffer_out_m(smooth_path.size(), 3);
  for (int i = 0; i < smooth_path.size(); i++) {
    buffer_out_m(i, 0) = buffer;
    buffer_out_m(i, 1) = buffer;
    buffer_out_m(i, 2) = 0;
  }
  meter_out_m -= buffer_out_m;
  */

  // PENDING...
  // convert to latitude and longitude
  // divide by rotation
  // transfer start's 0,0 to original position

  if (plot) {
    ::matplotlibcpp::xkcd();
    ::matplotlibcpp::plot(smooth_x, smooth_y);
    ::matplotlibcpp::plot(obs_x, obs_y);
    ::matplotlibcpp::xlim(0, 50);
    ::matplotlibcpp::ylim(0, 50);
    ::matplotlibcpp::show();
  }
}

}  // namespace RRT

int main(int argc, char **argv) {
  static struct option getopt_options[] = {{"plot", no_argument, 0, 'p'},
                                           {0, 0, 0, 0}};

  while (1) {
    int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
    if (opt == -1) break;

    switch (opt) {
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
