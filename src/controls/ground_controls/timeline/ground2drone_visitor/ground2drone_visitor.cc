#include "ground2drone_visitor.h"

namespace src {
namespace controls {
namespace ground_controls {
namespace timeline {
namespace ground2drone_visitor {

Ground2DroneVisitor::Ground2DroneVisitor() {}

bool Ground2DroneVisitor::Process(GroundProgram *input_program,
                                  DroneProgram &drone_program) {
  return Visit(input_program, drone_program);
}

void Ground2DroneVisitor::ConcatenateDroneProgramCommands(
    DroneProgram &base_program, DroneProgram new_program) {

  base_program.mutable_commands()->MergeFrom(new_program.commands());
}

// Visitors ////////////////////////////////////////////////////////////////////
bool Ground2DroneVisitor::Visit(GroundProgram *n, DroneProgram &drone_program) {
  // Visit all commands.
  for (int i = 0; i < n->commands_size(); i++) {
    GroundCommand *current_command = n->mutable_commands(i);
    DroneProgram command_program;
    bool success = Visit(current_command, command_program);
    ConcatenateDroneProgramCommands(drone_program, command_program);
    if (!success) {
      return false;
    }
  }

  return true;
}

bool Ground2DroneVisitor::Visit(GroundCommand *n, DroneProgram &drone_program) {
  DroneProgram command_program;

  // Select the specific command that this generic command type encloses, if
  // any.
  bool success = true;
  if (n->has_waypoint_command()) {
    success = success && Visit(n->mutable_waypoint_command(), drone_program);
  } else if (n->has_ugv_drop_command()) {
    success = success && Visit(n->mutable_ugv_drop_command(), drone_program);
  } else if (n->has_survey_command()) {
    success = success && Visit(n->mutable_survey_command(), drone_program);
  } else if (n->has_off_axis_command()) {
    success = success && Visit(n->mutable_off_axis_command(), drone_program);
  } else if (n->has_wait_command()) {
    success = success && Visit(n->mutable_wait_command(), drone_program);
  }

  ConcatenateDroneProgramCommands(drone_program, command_program);
  return success;
}

bool Ground2DroneVisitor::Visit(WaypointCommand *n,
                                DroneProgram &drone_program) {

  // Create a GotoCommand to fly to the waypoint while avoiding obstacles on the
  // field.
  bool success;
  {
    GotoCommand *goto_command = new GotoCommand();
    goto_command->mutable_goal()->CopyFrom(n->goal());
    DroneProgram goto_command_program;
    success = Visit(goto_command, goto_command_program);
    ConcatenateDroneProgramCommands(drone_program, goto_command_program);
  }

  return success;
}

bool Ground2DroneVisitor::Visit(UgvDropCommand *n,
                                DroneProgram &drone_program) {

  // TODO(Comran): Implement
  (void)n;
  (void)drone_program;
  return true;
}

bool Ground2DroneVisitor::Visit(SurveyCommand *n, DroneProgram &drone_program) {

  // TODO(Weber): Implement
  (void)n;
  (void)drone_program;

  return true;
}

bool Ground2DroneVisitor::Visit(OffAxisCommand *n,
                                DroneProgram &drone_program) {
  // TODO(Comran): Implement
  (void)n;
  (void)drone_program;

  return true;
}

bool Ground2DroneVisitor::Visit(WaitCommand *n, DroneProgram &drone_program) {
  {
    DroneCommand *sleep_cmd = new DroneCommand();
    SleepCommand *sleep = sleep_cmd->mutable_sleep_command();
    sleep->set_time(n->time());
    drone_program.mutable_commands()->AddAllocated(sleep_cmd);
  }

  return true;
}

bool Ground2DroneVisitor::Visit(GotoCommand *n, DroneProgram &drone_program) {

  // Go to a certain location on the field while avoiding obstacles by
  // calculating a safe path to travel and commanding the drone to follow this
  // path using TranslateCommand.

  // TODO(RyanT): Add obstacle avoidance calculations here. Currently, this
  // flies
  // directly to the goal.
  {
    DroneCommand *translate_cmd = new DroneCommand();
    TranslateCommand *translate = translate_cmd->mutable_translate_command();
    translate->set_allocated_goal(n->mutable_goal());
    drone_program.mutable_commands()->AddAllocated(translate_cmd);
  }

  return true;
}

} // namespace ground2drone_visitor
} // namespace timeline
} // namespace ground_controls
} // namespace controls
} // namespace src
