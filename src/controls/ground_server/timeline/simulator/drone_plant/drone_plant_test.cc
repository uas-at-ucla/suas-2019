#include "drone_plant.h"

#include <cmath>
#include <iomanip>
#include <iostream>

#include "gtest/gtest.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {
namespace drone_plant {
namespace testing {

TEST(DronePlantTest, GoFromPointAToPointB) {
  const int kLoopIteration = 100;
  //const int kPrintWidth = 12;

  DronePlant plant({0, 0, 0}, kLoopIteration);
  Position3D goal = {0.001, 0.001, 100};

  int iteration = 0;
  while (GetDistance2D(plant.position(), goal) > 0.1) {
    // Use a mock proportionality PID to direct the drone.
    Vector3D velocity_pid = PointTowards(plant.position(), goal);
    velocity_pid *=
        ::std::sqrt(::std::pow(GetDistance2D(plant.position(), goal), 2) +
                    ::std::pow(plant.position().altitude - goal.altitude, 2));
    velocity_pid *= 0.1;

    //::std::cout << velocity_pid.x << ::std::endl
    //            << velocity_pid.y << ::std::endl
    //            << velocity_pid.z << ::std::endl;

    Position3D start = plant.position();
    plant.MoveDrone(velocity_pid);
    Position3D end = plant.position();

    double distance_3d = GetDistance2D(start, end);
    distance_3d = ::std::sqrt(::std::pow(distance_3d, 2) +
                              ::std::pow(end.altitude - start.altitude, 2));
    double speed = distance_3d / (1.0 / kLoopIteration);

    //::std::cout << "Time: " << ::std::setw(kPrintWidth)
    //            << iteration * 1.0 / kLoopIteration
    //            << " Latitude: " << ::std::setw(kPrintWidth)
    //            << plant.position().latitude
    //            << " Longitude: " << ::std::setw(kPrintWidth)
    //            << plant.position().longitude
    //            << " Altitude: " << ::std::setw(kPrintWidth)
    //            << plant.position().altitude << " Speed: " << speed
    //            << ::std::endl;

    // Use the following for creating CSV plots of the data.
    ::std::cout << (iteration * 1.0 / kLoopIteration) << ", "
                << plant.position().latitude << ", "
                << plant.position().longitude << ", "
                << plant.position().altitude << ", " << speed << ::std::endl;

    if (iteration++ > 1e5) {
      FAIL() << "Simulated drone took to long to complete test!";
    }
  }
}

} // namespace testing
} // namespace drone_plant
} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
