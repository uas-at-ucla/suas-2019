#include "src/control/loops/pilot/pilot.h"

#include "gtest/gtest.h"

#include "lib/physics_structs/physics_structs.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {
namespace testing {

constexpr double kMetPositionTolerance = 0.5;

class PilotTest : public ::testing::Test {
 protected:
  PilotTest() {}

  bool CheckMetGoal(Position3D position, Position3D goal) {
    return GetDistance3D(position, goal) < kMetPositionTolerance;
  }

  void MetGoal(Position3D position, Position3D goal) {
    EXPECT_LE(GetDistance3D(position, goal), kMetPositionTolerance);
  }

  Pilot pilot_;
};

TEST_F(PilotTest, ReachesGoalTest) {
  constexpr double kLoopFrequency = 100;

  constexpr double kMetersPerCoordinate = 111132.954;
  Position3D drone_position = {0, 0, 0};
  Position3D goal = {0.0000002, 0.000003, 10};

  for (int i = 0;
       i < 100 * kLoopFrequency && !CheckMetGoal(drone_position, goal); i++) {
    Vector3D flight_direction = pilot_.Calculate(drone_position, goal);

    drone_position.latitude +=
        flight_direction.x / kLoopFrequency / kMetersPerCoordinate;
    drone_position.longitude +=
        flight_direction.y / kLoopFrequency / kMetersPerCoordinate;
    drone_position.altitude -= flight_direction.z / kLoopFrequency;
  }

  MetGoal(drone_position, goal);
}

}  // namespace testing
}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
