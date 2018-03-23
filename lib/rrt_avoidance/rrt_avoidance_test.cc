#include <gtest/gtest.h>
#include <vector>
#include "getopt.h"

#include "matplotlibcpp.h"

#include "lib/physics_structs/physics_structs.h"
#include "lib/rrt_avoidance/rrt_avoidance.h"

namespace lib {
namespace rrt_avoidance {
namespace testing {

bool plot = false;

TEST(RRTAvoidance, NoObstacles) {
  // Check that RRT takes a straight line path from position to goal.

  // TODO(comran): Write something to check that final path is the shortest it
  // can be.

  RRTAvoidance rrt_avoidance;

  Position3D start = {0, 0, 0};
  Position3D end = {1e-3, 1e-3, 0};
  ::std::vector<Obstacle> obstacles;

  ::std::vector<Position3D> avoidance_path =
      rrt_avoidance.Process(start, end, obstacles);

  ::std::vector<double> final_x, final_y;
  for (size_t i = 0; i < avoidance_path.size(); i++) {
    ::std::cout << avoidance_path[i].latitude << ", "
                << avoidance_path[i].longitude << ::std::endl;

    final_x.push_back(avoidance_path[i].latitude);
    final_y.push_back(avoidance_path[i].longitude);
  }

  if (plot) {
    ::matplotlibcpp::xkcd();
    ::matplotlibcpp::plot(final_x, final_y);
    ::matplotlibcpp::show();
  }
}

TEST(RRTAvoidance, AvoidsObstacles) {
  // Check that RRT does not intersect with obstacles.

  // TODO(comran): Write something to check that final path does not intersect
  // with obstacles (both vertices and edges).

  RRTAvoidance rrt_avoidance;

  Position3D start = {0, 0, 0};
  Position3D end = {1e-3, 1e-3, 0};
  ::std::vector<Obstacle> obstacles;

  for (int i = 0; i < 3; i++) {
    Position3D obstacle_position = {0 + i * 1e-3 / 4, 0 + i * 1e-3 / 4, 0};
    Obstacle obstacle = {obstacle_position, 20};

    obstacles.push_back(obstacle);
  }

  ::std::vector<Position3D> avoidance_path =
      rrt_avoidance.Process(start, end, obstacles);

  ::std::vector<double> final_x, final_y;
  for (size_t i = 0; i < avoidance_path.size(); i++) {
    ::std::cout << avoidance_path[i].latitude << ", "
                << avoidance_path[i].longitude << ::std::endl;

    final_x.push_back(avoidance_path[i].latitude);
    final_y.push_back(avoidance_path[i].longitude);
  }

  if (plot) {
    ::matplotlibcpp::xkcd();
    ::matplotlibcpp::plot(final_x, final_y);
    ::matplotlibcpp::show();
  }
}

// TODO(comran): Make this test not freeze after performing only a couple
// calculations.
TEST(RRTAvoidance, MultipleAvoidanceCalculations) {
  // Make sure we can use the same RRT avoidance class to perform multiple
  // calculations.

  RRTAvoidance rrt_avoidance;
  for (int calcs = 0; calcs < 20; calcs++) {
    ::std::cout << "Running calculation #" << calcs << ::std::endl;

    Position3D start = {0, 0, 0};
    Position3D end = {1e-3, 1e-3, 0};
    ::std::vector<Obstacle> obstacles;

    ::std::vector<double> obs_x, obs_y;
    for (int i = 1; i < 4; i++) {
      Position3D obstacle_position = {0 + i * 1e-3 / 4, 0 + i * 1e-3 / 4, 0};
      Obstacle obstacle = {obstacle_position, 5};

      for (double x = obstacle_position.latitude - 1e-4;
           x < obstacle_position.latitude + 1e-4; x += 1e-4) {
        for (double y = obstacle_position.longitude - 1e-4;
             y < obstacle_position.longitude + 1e-4; y += 1e-4) {
          ::std::cout << "x " << x << " y " << y << ::std::endl;
          obs_x.push_back(x);
          obs_y.push_back(y);
        }
      }

      obstacles.push_back(obstacle);
    }

    ::std::vector<Position3D> avoidance_path =
        rrt_avoidance.Process(start, end, obstacles);

    ::std::vector<double> final_x, final_y;
    for (size_t i = 0; i < avoidance_path.size(); i++) {
      ::std::cout << avoidance_path[i].latitude << ", "
                  << avoidance_path[i].longitude << ::std::endl;

      final_x.push_back(avoidance_path[i].latitude);
      final_y.push_back(avoidance_path[i].longitude);
    }

    if (plot) {
      ::matplotlibcpp::xkcd();
      ::matplotlibcpp::plot(final_x, final_y);
      ::matplotlibcpp::plot(obs_x, obs_y);
      ::matplotlibcpp::show();
    }
  }
}

}  // namespace testing
}  // namespace rrt_avoidance
}  // namespace lib

int main(int argc, char **argv) {
  // Add some command line options for displaying plots, if we want to see them.

  static struct option getopt_options[] = {{"plot", no_argument, 0, 'p'},
                                           {0, 0, 0, 0}};

  while (1) {
    int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
    if (opt == -1) break;

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
