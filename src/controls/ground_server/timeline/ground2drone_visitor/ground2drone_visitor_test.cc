#include "src/controls/ground_server/timeline/ground2drone_visitor/ground2drone_visitor.h"

#include "gtest/gtest.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace ground2drone_visitor {
namespace testing {

TEST(Ground2DroneVisitorTest, CanVisit) {
  GroundProgram input_instructions;
  for (int i = 0; i < 10; i++) {
    GroundCommand *cmd = input_instructions.add_commands();
    Position3D *goal = new Position3D();
    goal->set_latitude(i);
    goal->set_longitude(i * 2);
    goal->set_altitude(i * 3);

    cmd->mutable_waypoint_command()->set_allocated_goal(goal);
  }

  Ground2DroneVisitor ground_to_drone_visitor;
  DroneProgram drone_program =
      ground_to_drone_visitor.Process(&input_instructions);

  ASSERT_EQ(drone_program.commands_size(), 10);

  for (int i = 0; i < drone_program.commands_size(); i++) {
    DroneCommand drone_command = drone_program.commands()[i];

    ASSERT_TRUE(drone_command.has_translate_command());

    TranslateCommand *translate_command =
        drone_command.mutable_translate_command();

    ASSERT_EQ(translate_command->goal().latitude(), i);
    ASSERT_EQ(translate_command->goal().longitude(), i * 2);
    ASSERT_EQ(translate_command->goal().altitude(), i * 3);
  }
}

} // namespace testing
} // namespace ground2drone_visitor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
