#include "src/controls/ground_server/timeline/executor/executor.h"

#include "gtest/gtest.h"

#include "lib/physics_structs/physics_structs.h"

namespace src {
namespace controls {
namespace loops {
namespace executor {
namespace testing {
namespace {
const double kMetPositionTolerance = 5;
const double kMetersPerCoordinate = GetDistance2D({0, 0, 0}, {1, 0, 0});
} // namespace

class ExecutorPlant {
 public:
  ExecutorPlant(Position3D init_position, double loop_frequency) :
      drone_position_(init_position),
      loop_frequency_(loop_frequency) {}

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

class ExecutorTest : public ::testing::Test {
 protected:
  ExecutorTest() {}

  bool CheckMetGoal(Position3D position, Position3D goal) {
    return GetDistance3D(position, goal) < kMetPositionTolerance;
  }

  void MetGoal(Position3D position, Position3D goal) {
    EXPECT_LE(GetDistance3D(position, goal), kMetPositionTolerance);
  }

  Executor executor_;
};

TEST_F(ExecutorTest, ReachesGoalTest) {
  ExecutorPlant plant({0, 0, 0}, 100);
  Position3D goal = {0.0005, 0.0003, 10};

  ::lib::mission_manager::Mission mission;
  ::lib::mission_manager::GotoCommand *goto_cmd =
      mission.add_commands()->mutable_gotocommand();

  ::lib::mission_manager::Position3D *goto_goal =
      new ::lib::mission_manager::Position3D();
  goto_goal->set_latitude(goal.latitude);
  goto_goal->set_longitude(goal.longitude);
  goto_goal->set_altitude(goal.altitude);

  goto_cmd->set_allocated_goal(goto_goal);
  goto_cmd->set_come_to_stop(true);

  executor_.SetMission(mission);

  double runtime_in_seconds = 25;

  ::Eigen::Vector3d current_velocities(0, 0, 0);

  for (int i = 0; i < runtime_in_seconds * plant.GetLoopFrequency() &&
                  !CheckMetGoal(plant.GetPosition(), goal);
       i++) {
    usleep(1e3);
    ::std::cout << "drone at (" << plant.GetPosition().latitude << ", "
                << plant.GetPosition().longitude << ")\n";

    executor::ExecutorOutput flight_direction =
        executor_.Calculate(plant.GetPosition(), current_velocities);
    ::std::cout << flight_direction.flight_velocities.x << ", "
                << flight_direction.flight_velocities.y << ", "
                << flight_direction.flight_velocities.z << ::std::endl;

    current_velocities << flight_direction.flight_velocities.x,
        flight_direction.flight_velocities.y,
        flight_direction.flight_velocities.z;

    plant.MoveDrone(flight_direction.flight_velocities);
  }

  MetGoal(plant.GetPosition(), goal);
}

} // namespace testing
} // namespace executor
} // namespace loops
} // namespace controls
} // namespace src
