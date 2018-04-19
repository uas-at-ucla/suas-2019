#include "src/control/loops/pilot/pilot.h"

#include "gtest/gtest.h"

#include "lib/physics_structs/physics_structs.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {
namespace testing {
namespace {
const double kMetPositionTolerance = 3;
const double kMetersPerCoordinate = GetDistance2D({0, 0, 0}, {1, 0, 0});
}  // namespace

class PilotPlant {
 public:
  PilotPlant(Position3D init_position, double loop_frequency)
      : drone_position_(init_position), loop_frequency_(loop_frequency) {}

  void MoveDrone(Vector3D flight_direction) {
    drone_position_.latitude +=
        flight_direction.x / loop_frequency_ / kMetersPerCoordinate;
    drone_position_.longitude +=
        flight_direction.y / loop_frequency_ / kMetersPerCoordinate;
    drone_position_.altitude -= flight_direction.z / loop_frequency_;
  }

  Position3D GetPosition() { return drone_position_; }

  double GetLoopFrequency() { return loop_frequency_; }

 private:
  Position3D drone_position_;
  double loop_frequency_;
};

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
  PilotPlant plant({0, 0, 0}, 100);
  Position3D goal = {0.0002, 0.0003, 10};

  ::lib::mission_manager::Mission mission;
  ::lib::mission_manager::GotoCommand *goto_cmd =
      mission.add_commands()->mutable_gotocommand();

  ::lib::mission_manager::Position3D *goto_goal =
      new ::lib::mission_manager::Position3D();
  goto_goal->set_latitude(goal.latitude);
  goto_goal->set_longitude(goal.longitude);
  goto_goal->set_altitude(goal.altitude);

  goto_cmd->set_allocated_goal(goto_goal);

  pilot_.SetMission(mission);

  double runtime_in_seconds = 10;

  for (int i = 0; i < runtime_in_seconds * plant.GetLoopFrequency() &&
                  !CheckMetGoal(plant.GetPosition(), goal);
       i++) {
    usleep(1e3);
    ::std::cout << "drone at (" << plant.GetPosition().latitude << ", "
                << plant.GetPosition().longitude << ")\n";

    pilot::PilotOutput flight_direction = pilot_.Calculate(plant.GetPosition());

    plant.MoveDrone(flight_direction.flight_velocities);
  }

  MetGoal(plant.GetPosition(), goal);
}

}  // namespace testing
}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
