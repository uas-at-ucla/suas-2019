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
  const int kPrintWidth = 12;
  const int kPrintDecimals = 6;
  Battery battery(32,32,.125,0);

  DronePlant plant({0, 0, 0}, kLoopIteration,battery);
  Position3D goal = {0.001, 0.001, 100};
  
  int iteration = 0;
  while (GetDistance2D(plant.position(), goal) > 0.1) {
    // Use a mock proportionality PID to direct the drone.
    Vector3D velocity_pid = PointTowards(plant.position(), goal);
    velocity_pid *=
        ::std::sqrt(::std::pow(GetDistance2D(plant.position(), goal), 2) +
                    ::std::pow(plant.position().altitude - goal.altitude, 2));
    velocity_pid *= 0.1;

    Position3D start = plant.position();
    plant.MoveDrone(velocity_pid);
    Position3D end = plant.position();
    Battery current=plant.battery();

    double distance_3d = GetDistance2D(start, end);
    distance_3d = ::std::sqrt(::std::pow(distance_3d, 2) +
                              ::std::pow(end.altitude - start.altitude, 2));
    double speed = distance_3d / (1.0 / kLoopIteration);

    ::std::cout << "Time: " << ::std::fixed
                << ::std::setprecision(kPrintDecimals)
                << ::std::setw(kPrintWidth) << iteration * 1.0 / kLoopIteration
                << " Latitude: " << ::std::setw(kPrintWidth)
                << plant.position().latitude
                << " Longitude: " << ::std::setw(kPrintWidth)
                << plant.position().longitude
                << " Altitude: " << ::std::setw(kPrintWidth)
                << plant.position().altitude << " Speed: " << speed
                << " Battery Remaining: "<<std::setw(kPrintWidth)
                << current.GetRemainingCapacity()
                << ::std::endl;

    if (iteration++ > 1e5) {
      FAIL() << "Simulated drone took to long to complete test!";
    }
    else if(!current.CanSourceCurrent()){
      FAIL() << "The drone ran out of battery at " << iteration*1.0 / kLoopIteration;
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
