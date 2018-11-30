#include "executor.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace executor {

Executor::Executor() {}

DroneProgram Executor::Process(DroneProgram *input_program) {
  return Visit(input_program);
}

void Executor::ConcatenateDroneProgramCommands(
    DroneProgram &base_program, DroneProgram new_program) {

  base_program.mutable_commands()->MergeFrom(new_program.commands());
}

// Visitors ////////////////////////////////////////////////////////////////////
DroneProgram Executor::Visit(GroundProgram *n) {
  DroneProgram drone_program;

  // Visit all commands.
  for (int i = 0; i < n->commands_size(); i++) {
    GroundCommand *current_command = n->mutable_commands(i);

    DroneProgram command_program = Visit(current_command);
    ConcatenateDroneProgramCommands(drone_program, command_program);
  }

  return drone_program;
}

DroneProgram Executor::Visit(GroundCommand *n) {
  DroneProgram drone_program;
  DroneProgram command_program;

  // Select the specific command that this generic command type encloses, if
  // any.
  if (n->has_translate_command()) {
    command_program = Visit(n->mutable_translate_command());
  } else if (n->has_ugv_drop_command()) {
    command_program = Visit(n->mutable_ugv_drop_command());
  } else if (n->has_survey_command()) {
    command_program = Visit(n->mutable_survey_command());
  } else if (n->has_off_axis_command()) {
    command_program = Visit(n->mutable_off_axis_command());
  } else if (n->has_wait_command()) {
    command_program = Visit(n->mutable_wait_command());
  }

  ConcatenateDroneProgramCommands(drone_program, command_program);

  return drone_program;
}

DroneProgram Executor::Visit(TranslateCommand *n) {
  DroneProgram drone_program;

  // TODO(Comran): Implement
  (void)n;

  return drone_program;
}

DroneProgram Executor::Visit(UgvDropCommand *n) {
  DroneProgram drone_program;

  // TODO(Comran): Implement
  (void)n;

  return drone_program;
}

DroneProgram Executor::Visit(SurveyCommand *n) {
  DroneProgram drone_program;

  // TODO(Weber): Implement
  (void)n;

  return drone_program;
}

DroneProgram Executor::Visit(OffAxisCommand *n) {
  DroneProgram drone_program;

  // TODO(Comran): Implement
  (void)n;

  return drone_program;
}

DroneProgram Executor::Visit(WaitCommand *n) {
  DroneProgram drone_program;

  {
    DroneCommand *sleep_cmd = new DroneCommand();
    SleepCommand *sleep = sleep_cmd->mutable_sleep_command();
    sleep->set_time(n->time());
    drone_program.mutable_commands()->AddAllocated(sleep_cmd);
  }

  return drone_program;
}

DroneProgram Executor::Visit(GotoCommand *n) {
  DroneProgram drone_program;

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

  return drone_program;
}

} // namespace executor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
