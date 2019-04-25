#include "src/controls/ground_server/timeline/context_visitors/context_visitor.h"

#include "gtest/gtest.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {
namespace testing {
using namespace lib::mission_manager;
namespace {
const int num_field_points = 4;
const int num_test_points = 5;

::std::string serialized_instructions;
double field_boundary[num_field_points][2] = {
    {0, 0},
    {0, 300},
    {300, 300},
    {300, 0}}; // points used to initialize as field boundaries
double test_points[num_test_points][2] = {
    {0.5, 0.5},
    {1, 4000},
    {-1, -2},
    {0, 0},
    {0.7, 0.7}}; // points used to test the field boundary functions
bool test_result[num_test_points] = {
    true, false, false, false,
    true}; // whether each point should be in field boundary

lib::mission_manager::Position3D start_location;
} // namespace

void initInputInstructionsObject(GroundProgram &input_instructions) {
  // initialize program with field boundary
  for (int i = 0; i < num_field_points; i++) {
    Position2D *p = input_instructions.add_field_boundary();
    p->set_latitude(field_boundary[i][0]);
    p->set_longitude(field_boundary[i][1]);
  }
}

TEST(ContextVisitorTest, CanDetectOutOfBoundary) {
  ContextVisitor context_visitor;
  GroundProgram input_instructions;
  initInputInstructionsObject(input_instructions);
  start_location.set_altitude(0.1);
  start_location.set_longitude(0.1);
  start_location.set_latitude(0.1);

  // test if throws the correct exception when given a waypoint goal that's out
  // of bounds
  for (int i = 0; i < num_test_points; i++) {
    input_instructions.clear_commands();
    GroundCommand *cmd = input_instructions.add_commands();
    lib::mission_manager::Position3D *goal =
        new lib::mission_manager::Position3D();
    goal->set_latitude(test_points[i][0]);
    goal->set_longitude(test_points[i][1]);
    goal->set_altitude(i * 3);
    cmd->mutable_waypoint_command()->set_allocated_goal(goal);
    input_instructions.SerializeToString(&serialized_instructions);
    try {
      context_visitor.Process(serialized_instructions, start_location);

      if (test_result[i] == false) {
        // if should throw exception but didn't throw
        FAIL() << "Expected destination out of bounds";
      }
    } catch (const char *msg) {
      ASSERT_EQ(msg, "destination out of bounds");
    } catch (...) {
      // if should throw exception but threw a different type
      FAIL() << "Expected destination out of bounds";
    }
  }
}
TEST(ContextVisitorTest, CanDetectIncorrectParsing) {
  ContextVisitor context_visitor;
  ::std::string serialized_instructions =
      "abcde"; // a string that cannot be de-serialized

  try {
    context_visitor.Process(serialized_instructions, start_location);
  } catch (const char *msg) {
    ASSERT_EQ(msg, "cannot parse input string");
  } catch (...) {
    // if should throw exception but threw a different type
    FAIL() << "Expected cannot parse input string";
  }
}

TEST(ContextVisitorTest, CanPerformObjectAvoidance) {
  ContextVisitor context_visitor;
  GroundProgram input_instructions;
  initInputInstructionsObject(input_instructions);

  Position2D *obstacle_center = new Position2D;
  obstacle_center->set_latitude(20);
  obstacle_center->set_longitude(20);

  // create a mission to fly to goal with a small obstacle
  lib::mission_manager::Position3D *goal = new lib::mission_manager::Position3D;
  goal->set_latitude(80);
  goal->set_longitude(80);
  goal->set_altitude(0);

  GroundCommand *cmd = input_instructions.add_commands();
  cmd->mutable_waypoint_command()->set_allocated_goal(goal);
  StaticObstacle *obstacle = input_instructions.add_static_obstacles();
  obstacle->set_cylinder_radius(10);
  obstacle->set_allocated_location(obstacle_center);

  // add second obstacle
  obstacle = input_instructions.add_static_obstacles();
  Position2D *center2 = new Position2D;
  center2->set_latitude(60);
  center2->set_longitude(60);
  obstacle->set_allocated_location(center2);
  obstacle->set_cylinder_radius(10);

  input_instructions.SerializeToString(&serialized_instructions);
  // expect that the planned path will be within the boundaries
  try {
    context_visitor.Process(serialized_instructions, start_location);

    // get and print the obstacle avoidance path
    ::std::vector<lib::mission_manager::Position3D> avoidance_path =
        context_visitor.getAvoidancePath();
    ::std::cout << "calculated object avoidance path: " << ::std::endl;
    for (auto const &step : avoidance_path) {
      ::std::cout << step.longitude() << ' ' << step.latitude() << ::std::endl;
    }
  } catch (const std::exception &e) {
    FAIL() << "Expected no exceptions";
  }
}

// current test not working, rrt avoidance appears to output the same result
// with same obstacles of different cylinder_radius

// TEST(ContextVisitorTest, CanDetectObjectAvoidanceOutOfBounds) {
// ContextVisitor context_visitor;
//   GroundProgram input_instructions;
//   initInputInstructionsObject(input_instructions);

//   Position2D *obstacle_center = new Position2D;
//   obstacle_center->set_latitude(20);
//   obstacle_center->set_longitude(20);

//   // create a mission to fly to 9, 9, with a big obstacle
//   Position3D *goal = new Position3D;
//   goal->set_latitude(80);
//   goal->set_longitude(80);
//   goal->set_altitude(0);

//   GroundCommand *cmd = input_instructions.add_commands();
//   cmd->mutable_waypoint_command()->set_allocated_goal(goal);
//   StaticObstacle *obstacle = input_instructions.add_static_obstacles();
//   obstacle->set_allocated_location(obstacle_center);
//   obstacle->set_cylinder_radius(10);

//   obstacle = input_instructions.add_static_obstacles();
//   Position2D *center2 = new Position2D;
//   center2->set_latitude(60);
//   center2->set_longitude(60);
//   obstacle->set_allocated_location(center2);
//   obstacle->set_cylinder_radius(10);

//   input_instructions.SerializeToString(&serialized_instructions);

//   // expect to go out of bounds, since obstacle is too large
//   bool exception_thrown = false;
//   try {
//     context_visitor.Process(serialized_instructions, start_location);
//   } catch (char *msg) {
//     exception_thrown = true;
//     ASSERT_EQ(msg, "calculated avoidance path out of bounds");
//   }
//   if (!exception_thrown) {
//     FAIL() << "Expected to throw 'avoidance path out of bounds' exception,
//     but no exception was thrown ";
//   }
// }
} // namespace testing

} // namespace context_visitors
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
