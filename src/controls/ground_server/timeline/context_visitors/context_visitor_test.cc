#include "src/controls/ground_server/timeline/context_visitors/context_visitor.h"

#include "gtest/gtest.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {
namespace testing {

TEST(ContextVisitorTest, CanDetectOutOfBoundary) {
  GroundProgram input_instructions;
  const int num_field_points = 4;
  const int num_test_points = 5;
  double field_boundary[num_field_points][2] = {
      {0, 0},
      {0, 1},
      {1, 1},
      {1, 0}}; // points used to initialize as field boundaries
  double test_points[num_test_points][2] = {
      {0.5, 0.5},
      {1, 2},
      {-1, -2},
      {0, 0},
      {0.7, 0.7}}; // points used to test the field boundary functions
  bool test_result[num_test_points] = {
      true, false, false, false,
      true}; // whether each point should be in field boundary
  for (int i = 0; i < num_field_points; i++) {
    Position2D *p = input_instructions.add_field_boundary();
    p->set_latitude(field_boundary[i][0]);
    p->set_longitude(field_boundary[i][1]);
  }

  context_visitors::ContextVisitor context_visitor;
  ::std::string serialized_instructions;

  // test if throws the correct exception when given a waypoint goal that's out
  // of bounds
  for (int i = 0; i < num_test_points; i++) {
    input_instructions.clear_commands();
    GroundCommand *cmd = input_instructions.add_commands();
    Position3D *goal = new Position3D();
    goal->set_latitude(test_points[i][0]);
    goal->set_longitude(test_points[i][1]);
    goal->set_altitude(i * 3);
    cmd->mutable_waypoint_command()->set_allocated_goal(goal);
    input_instructions.SerializeToString(&serialized_instructions);
    try {
      context_visitor.Process(serialized_instructions);
      
      if (test_result[i] == false) {
        // if should throw exception but didn't throw
        FAIL() << "Expected going out of bounds";
      }
    } catch (const char *msg) {
      ASSERT_EQ(msg, "going out of bounds");
    } catch (...) {
      // if should throw exception but threw a different type
      FAIL() << "Expected going out of bounds";
    }
  }
}
TEST(ContextVisitorTest, CanDetectIncorrectParsing) {
  ContextVisitor context_visitor;
  ::std::string serialized_instructions =
      "abcde"; // a string that cannot be de-serialized

  try {
    context_visitor.Process(serialized_instructions);
  } catch (const char *msg) {
    ASSERT_EQ(msg, "cannot parse input string");
  } catch (...) {
    // if should throw exception but threw a different type
    FAIL() << "Expected cannot parse input string";
  }
}

} // namespace testing
} // namespace context_visitors
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
