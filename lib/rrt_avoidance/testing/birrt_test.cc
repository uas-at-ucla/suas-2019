#include "getopt.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>

#include "lib/physics_structs/physics_structs.h"
#include "lib/rrt_avoidance/2dplane/2dplane.hpp"
#include "lib/rrt_avoidance/2dplane/GridStateSpace.hpp"
#include "lib/rrt_avoidance/birrt/birrt.h"
#include "lib/rrt_avoidance/planning/Path.hpp"
#include "matplotlibcpp.h"
#include <cmath>
#include <math.h>
#include <stdio.h>

const int DISPLAY_MODE = 1;

using namespace std;
using namespace Eigen;

namespace lib {
namespace rrt_avoidance {
namespace testing {

bool plot = false;

TEST(BiRRT, Instantiation) {
  BiRRT biRRT(make_shared<GridStateSpace>(100, 100, 100, 100), hash,
              dimensions);
}

TEST(BiRRT, GeocoordToObstacleGridTest) {
  for (int tests = 0; tests < 50; tests++) {
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
    for (int i = 2; i < rows; i++) {
      m(i, 0) = 2e-3;
      m(i, 1) = 1.6e-3 + (i - 2) * 1.2e-3;
      m(i, 2) = 50;
    }
    ::std::cout << m << ::std::endl;

    // Shift everything so that the drone position at the origin.
    ::Eigen::MatrixXd m_trans(rows, 3);
    m_trans = m.row(0).replicate(rows, 1);
    m -= m_trans;

    ::std::cout << m << ::std::endl;

    // Make the start->end points exist in the first quadrant.
    ::Eigen::MatrixXd reflection_m = MatrixXd::Identity(3, 3);
    reflection_m(0, 0) = (m(1, 0) >= 0) ? 1 : -1;
    reflection_m(1, 1) = (m(1, 1) >= 0) ? 1 : -1;
    m *= reflection_m;

    // Convert coordinate components to meters.
    double coordinate_to_meter = GetDistance2D({0, 0, 0}, {1, 0, 0});

    for (int i = 0; i < rows; i++) {
      m(i, 0) = m(i, 0) * coordinate_to_meter;
      m(i, 1) = m(i, 1) * coordinate_to_meter;
    }

    // Scale everything to be within the 0 -> 50 obstacle grid used by RRT.
    const int kDimension = 50;
    double meter_width = m(1, 0);
    double meter_height = m(1, 1);
    double longest_dim =
        meter_width > meter_height ? meter_width : meter_height;

    double scale = (kDimension - 2) / longest_dim;
    m *= scale;

    // Leave a 1 unit buffer around everything to make rrt more stable.
    ::Eigen::MatrixXd m_shift(rows, 3);
    ::Eigen::MatrixXd m_shift_row(1, 3);
    m_shift_row << 1, 1, 0;
    m_shift = m_shift_row.replicate(rows, 1);
    m += m_shift;

    std::shared_ptr<GridStateSpace> state_space = make_shared<GridStateSpace>(
        kDimension, kDimension, kDimension, kDimension);

    // Set start and end goals.
    Vector2d start = {m(0, 0), m(0, 1)};
    Vector2d goal = {m(1, 0), m(1, 1)};

    BiRRT biRRT(state_space, hash, dimensions);
    vector<double> obs_x, obs_y;

    // Draw obstacles as circles on the obstacle grid.
    for (int i = 2; i < rows; i++) {
      int obstacle_radius = m(i, 2);
      for (int offset_x = -obstacle_radius; offset_x <= obstacle_radius;
           offset_x++) {
        int y_max = sqrt(pow(obstacle_radius, 2) - pow(offset_x, 2)) + 1;

        for (int offset_y = -y_max; offset_y <= y_max; offset_y++) {
          int x = m(i, 0) + offset_x;
          int y = m(i, 1) + offset_y;
          if (x < 0 || x >= kDimension | y < 0 || y >= kDimension) {
            continue;
          }

          state_space->obstacleGrid().obstacleAt(Vector2i(x, y)) = true;

          obs_x.push_back(x);
          obs_y.push_back(y);
        }
      }
    }

    // Run the RRT.
    biRRT.set_start_state(start);
    biRRT.set_goal_state(goal);
    biRRT.set_step_size(1);
    biRRT.set_max_iterations(10000);
    int num_iterations = biRRT.Run();
    vector<Vector2d> path = biRRT.GetPath();
    vector<Vector2d> smooth_path(path);
    SmoothPath<Vector2d>(smooth_path, *state_space);

    ::Eigen::MatrixXd m_rrt_path(smooth_path.size(), 2);

    vector<double> smooth_x, smooth_y;
    for (int i = 0; i < smooth_path.size(); i++) {
      m_rrt_path(i, 0) = smooth_path[i][0];
      m_rrt_path(i, 1) = smooth_path[i][1];

      smooth_x.push_back(smooth_path[i][0]);
      smooth_y.push_back(smooth_path[i][1]);
    }

    cout << m_rrt_path << endl;

    // Apply inverse operations to get back to original coordinates.
    m_rrt_path -= m_shift.block<1, 2>(0, 0).replicate(smooth_path.size(), 1);
    m_rrt_path /= scale;
    m_rrt_path /= coordinate_to_meter;
    m_rrt_path *= reflection_m.block<2, 2>(0, 0);
    m_rrt_path += m_trans.block<1, 2>(0, 0).replicate(smooth_path.size(), 1);

    vector<double> final_x, final_y;
    for (int i = 0; i < smooth_path.size(); i++) {
      final_x.push_back(m_rrt_path(i, 0));
      final_y.push_back(m_rrt_path(i, 1));
    }

    ::std::cout << m_rrt_path << ::std::endl;

    if (plot) {
      ::matplotlibcpp::xkcd();
      ::matplotlibcpp::plot(smooth_x, smooth_y);
      ::matplotlibcpp::plot(obs_x, obs_y);
      ::matplotlibcpp::xlim(0, kDimension);
      ::matplotlibcpp::ylim(0, kDimension);
      ::matplotlibcpp::title("RRT Path in ObstacleGrid units");
      ::matplotlibcpp::show();

      ::matplotlibcpp::xkcd();
      ::matplotlibcpp::plot(final_x, final_y);
      ::matplotlibcpp::title("RRT Path in real-world geocoords");
      ::matplotlibcpp::show();
    }
  }
}

TEST(BiRRT, GetPath) {
  Vector2d start = {1, 1}, goal = {49, 49};

  std::shared_ptr<GridStateSpace> state_space =
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

  BiRRT biRRT(state_space, hash, dimensions);
  biRRT.set_start_state(start);
  biRRT.set_goal_state(goal);
  biRRT.set_step_size(1);
  biRRT.set_max_iterations(10000);

  int num_iterations = biRRT.Run();
  ASSERT_GT(num_iterations, 0);

  vector<Vector2d> path = biRRT.GetPath();
  vector<Vector2d> smooth_path(path);
  SmoothPath<Vector2d>(smooth_path, *state_space);

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

} // namespace testing
} // namespace rrt_avoidance
} // namespace lib

int main(int argc, char **argv) {
  static struct option getopt_options[] = {{"plot", no_argument, 0, 'p'},
                                           {0, 0, 0, 0}};

  while (1) {
    int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
    if (opt == -1)
      break;

    switch (opt) {
      case 'p':
        ::lib::rrt_avoidance::testing::plot = true;
        break;
      default:
        exit(1);
        break;
    }
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
