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

  for (int iteration = 0; iteration < 10; iteration++) {
    // Try going from start to end with no obstacles.

    Position3D start = {0, 0, 0};
    Position3D end = {1e-3, 1e-3, 0};

    ::lib::mission_manager::Obstacles obstacles;

    ::std::vector<Position3D> avoidance_path =
        rrt_avoidance.Process(start, end, obstacles);

    // Check that the result is a straight line with only two waypoints.
    double tolerance = 1e-8;
    ASSERT_EQ(avoidance_path.size(), 2);
    ASSERT_GE(avoidance_path[0].latitude, start.latitude - tolerance);
    ASSERT_LE(avoidance_path[0].latitude, start.latitude + tolerance);
    ASSERT_GE(avoidance_path[1].longitude, end.longitude - tolerance);
    ASSERT_LE(avoidance_path[1].longitude, end.longitude + tolerance);
  }
}

TEST(RRTAvoidance, DodgeObstacles) {
  // Check that RRT does not intersect with obstacles.

  // TODO(comran): Write something to check that final path does not intersect
  // with obstacles (both vertices and edges).

  RRTAvoidance rrt_avoidance;

  for (int calcs = 0; calcs < 10; calcs++) {
    ::std::vector<double> obs_x, obs_y;

    ::std::cout << "Running calculation #" << calcs << ::std::endl;

    Position3D start = {3e-3, 3e-3, 0};
    Position3D end = {1e-3, 1e-3, 0};
    ::lib::mission_manager::Obstacles obstacles;
    const double coordinate_to_meter = GetDistance2D({0, 0, 0}, {1, 0, 0});

    for (int i = 0; i < 3; i++) {
      ::lib::mission_manager::StaticObstacle *obstacle =
          obstacles.add_static_obstacles();
      ::lib::mission_manager::Position2D *obstacle_position =
          obstacle->mutable_location();
      obstacle_position->set_latitude(1e-3 + rand() % 4 * 1.2e-3);
      obstacle_position->set_longitude(1.6e-3 + i * 1.2e-3);
      obstacle->set_cylinder_radius(50);

      double coord_radius = obstacle->cylinder_radius() / coordinate_to_meter;
      for (double x = obstacle_position->latitude() - coord_radius;
           x < obstacle_position->latitude() + coord_radius;
           x += coord_radius / 20) {
        double y_max =
            sqrt(pow(coord_radius, 2) - pow(x - obstacle_position->latitude(), 2));

        for (double y = obstacle_position->longitude() - y_max;
             y < obstacle_position->longitude() + y_max; y += coord_radius / 20) {
          obs_x.push_back(x);
          obs_y.push_back(y);
        }
      }
    }

    ::std::vector<Position3D> avoidance_path =
        rrt_avoidance.Process(start, end, obstacles);

    ::std::vector<double> final_x, final_y;
    double tolerance = 1e-3;
    EXPECT_GE(avoidance_path.size(), 2);

    // Check that we start at the right place.
    EXPECT_GE(avoidance_path[0].latitude, start.latitude - 1e-3);
    EXPECT_LE(avoidance_path[0].latitude, start.latitude + 1e-3);

    // Check that we met the goal.
    EXPECT_GE(avoidance_path[avoidance_path.size() - 1].longitude,
              end.longitude - 1e-3);
    EXPECT_LE(avoidance_path[avoidance_path.size() - 1].longitude,
              end.longitude + 1e-3);

    //  // CIRCLE LINE SEGMENT COLLISION DETECTION
    //  for (size_t i = 1; i < avoidance_path.size(); i++) {
    //    // order of lat, long
    //    ::std::vector<double> AB;
    //    AB.push_back(avoidance_path.at(i).latitude -
    //                 avoidance_path.at(i - 1).latitude);
    //    AB.push_back(avoidance_path.at(i).longitude -
    //                 avoidance_path.at(i - 1).longitude);
    //    double magAB = sqrt(AB.at(0) * AB.at(0) + AB.at(1) * AB.at(1));

    //    for (size_t j = 0; j < obstacles.size(); j++) {
    //      ::std::vector<double> AC;
    //      AC.push_back(obstacles.at(j).position.latitude -
    //                   avoidance_path.at(i - 1).latitude);
    //      AC.push_back(obstacles.at(j).position.longitude -
    //                   avoidance_path.at(i - 1).longitude);
    //      double magAC = sqrt(AC.at(0) * AC.at(0) + AC.at(1) * AC.at(1));
    //      double R = obstacles.at(j).radius;
    //      double meter_to_coordinate = 1 / GetDistance2D({0, 0, 0}, {1, 0,
    //      0});

    //      double projFactor =
    //          (AC.at(0) * AB.at(0) + AC.at(1) * AB.at(1)) / (magAB * magAB);
    //      ::std::vector<double> AD;
    //      AD.push_back(projFactor * AB.at(0));
    //      AD.push_back(projFactor * AB.at(1));
    //      double magAD = sqrt(AD.at(0) * AD.at(0) + AD.at(1) * AD.at(1));

    //      double magCD = sqrt(magAC * magAC - magAD * magAD);
    //      ASSERT_GT(magCD, R * meter_to_coordinate);
    //    }
    //  }

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

// TODO(comran): Make this test not freeze after performing only a couple
// calculations.
TEST(RRTAvoidance, RandomObstacle) {
  // Make sure we can use the same RRT avoidance class to perform multiple
  // calculations.

  RRTAvoidance rrt_avoidance;
  const double coordinate_to_meter = GetDistance2D({0, 0, 0}, {1, 0, 0});

  for (int calcs = 0; calcs < 20; calcs++) {
    ::std::cout << "Running calculation #" << calcs << ::std::endl;

    Position3D start = {0.0, 0.0, 0};
    Position3D end = {1e3 / coordinate_to_meter, 1e3 / coordinate_to_meter, 0};
    ::lib::mission_manager::Obstacles obstacles;
    ::lib::mission_manager::StaticObstacle *obstacle =
        obstacles.add_static_obstacles();
    ::lib::mission_manager::Position2D *obstacle_position =
        obstacle->mutable_location();
    obstacle->set_cylinder_radius(150);

    ::std::vector<double> obs_x, obs_y;
    double rand_point = rand() / (RAND_MAX + 1.);
    obstacle_position->set_latitude((300 + 400 * rand_point) /
                                    coordinate_to_meter);
    obstacle_position->set_longitude((300 + 400 * rand_point) /
                                     coordinate_to_meter);

    // Draw circular obstacles on plot.
    double coord_radius = obstacle->cylinder_radius() / coordinate_to_meter;
    for (double x = obstacle_position->latitude() - coord_radius;
         x < obstacle_position->latitude() + coord_radius;
         x += coord_radius / 20) {
      double y_max =
          sqrt(pow(coord_radius, 2) - pow(x - obstacle_position->latitude(), 2));

      for (double y = obstacle_position->longitude() - y_max;
           y < obstacle_position->longitude() + y_max; y += coord_radius / 20) {
        obs_x.push_back(x);
        obs_y.push_back(y);
      }
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

    double tolerance = 1e-8;
    ASSERT_GE(avoidance_path[0].latitude, start.latitude - tolerance);
    ASSERT_LE(avoidance_path[0].latitude, start.latitude + tolerance);
    ASSERT_GE(avoidance_path[avoidance_path.size() - 1].longitude,
              end.longitude - tolerance);
    ASSERT_LE(avoidance_path[avoidance_path.size() - 1].longitude,
              end.longitude + tolerance);

//  if (plot) {
//    ::matplotlibcpp::xkcd();
//    ::matplotlibcpp::plot(final_x, final_y);
//    ::matplotlibcpp::plot(obs_x, obs_y);
//    ::matplotlibcpp::show();
//  }
  }
}

//TEST(RRTAvoidance, RealObstacles) {
//  RRTAvoidance rrt_avoidance;

//  Position3D start = {34.1747796812899 - 1e-3, -118.48062618357 - 1e-3, 0};
//  Position3D end = {34.1747796812899 + 1e-3, -118.48062618357 + 1e-3, 0};
//  ::lib::mission_manager::Obstacles obstacles;

//  ::lib::mission_manager::StaticObstacle *obstacle =
//      obstacles.add_static_obstacles();
//  ::lib::mission_manager::Position2D *obstacle_position =
//      obstacle->mutable_location();
//  obstacle_position->set_latitude(34.1747796812899);
//  obstacle_position->set_longitude(-118.48062618357);
//  obstacle->set_cylinder_radius(20);

//  obstacle_position->set_latitude(start.latitude);
//  obstacle_position->set_longitude(start.longitude);
//  obstacle->set_cylinder_radius(20);

//  ::std::vector<Position3D> avoidance_path =
//      rrt_avoidance.Process(start, end, obstacles);

//  ::std::vector<double> final_x, final_y;
//  double tolerance = 1e-3;
//  EXPECT_GT(avoidance_path.size(), 2);

//  // Check that we start at the right place.
//  ASSERT_GE(avoidance_path[0].latitude, start.latitude - 1e-3);
//  ASSERT_LE(avoidance_path[0].latitude, start.latitude + 1e-3);

//  // Check that we met the goal.
//  ASSERT_GE(avoidance_path[avoidance_path.size() - 1].longitude,
//            end.longitude - 1e-3);
//  ASSERT_LE(avoidance_path[avoidance_path.size() - 1].longitude,
//            end.longitude + 1e-3);

//  for (size_t i = 0; i < avoidance_path.size(); i++) {
//    ::std::cout << avoidance_path[i].latitude << ", "
//                << avoidance_path[i].longitude << ::std::endl;

//    final_x.push_back(avoidance_path[i].latitude);
//    final_y.push_back(avoidance_path[i].longitude);
//  }

//  if (plot) {
//    ::matplotlibcpp::xkcd();
//    ::matplotlibcpp::plot(final_x, final_y);
//    ::matplotlibcpp::show();
//  }
//}


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
