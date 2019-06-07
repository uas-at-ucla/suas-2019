#include "ground2drone_visitor.h"

namespace src {
namespace controls {
namespace ground_controls {
namespace timeline {
namespace ground2drone_visitor {

Ground2DroneVisitor::Ground2DroneVisitor() :
    ground_program_(nullptr),
    current_position_(nullptr) {}

DroneProgram Ground2DroneVisitor::Process(GroundProgram *input_program) {
  ground_program_ = input_program;

  current_position_ = nullptr;

  return Visit(input_program);
}

void Ground2DroneVisitor::ConcatenateDroneProgramCommands(
    DroneProgram &base_program, DroneProgram new_program) {

  base_program.mutable_commands()->MergeFrom(new_program.commands());
}

// Visitors ////////////////////////////////////////////////////////////////////
DroneProgram Ground2DroneVisitor::Visit(GroundProgram *n) {
  DroneProgram drone_program;

  // Visit all commands.
  for (int i = 0; i < n->commands_size(); i++) {
    GroundCommand *current_command = n->mutable_commands(i);

    DroneProgram command_program = Visit(current_command);
    ConcatenateDroneProgramCommands(drone_program, command_program);
  }

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(GroundCommand *n) {
  DroneProgram drone_program;
  DroneProgram command_program;

  // Select the specific command that this generic command type encloses, if
  // any.
  if (n->has_fly_through_command()) {
    command_program = Visit(n->mutable_fly_through_command());
  } else if (n->has_waypoint_command()) {
    command_program = Visit(n->mutable_waypoint_command());
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

DroneProgram Ground2DroneVisitor::Visit(FlyThroughCommand *n) {
  DroneProgram drone_program;

  // Create a GotoCommand to fly to the waypoint while avoiding obstacles on the
  // field.
  {
    GotoCommand *goto_command = new GotoCommand();
    goto_command->mutable_goal()->CopyFrom(n->goal());
    goto_command->set_come_to_stop(false);
    goto_command->mutable_goal()->set_altitude(goto_command->goal().altitude() /
                                               kFeetPerMeter);

    DroneProgram goto_command_program = Visit(goto_command);
    ConcatenateDroneProgramCommands(drone_program, goto_command_program);
  }

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(WaypointCommand *n) {
  DroneProgram drone_program;

  // Create a GotoCommand to fly to the waypoint while avoiding obstacles on the
  // field.
  {
    GotoCommand *goto_command = new GotoCommand();
    goto_command->mutable_goal()->CopyFrom(n->goal());
    goto_command->set_come_to_stop(true);
    goto_command->mutable_goal()->set_altitude(goto_command->goal().altitude() /
                                               kFeetPerMeter);

    DroneProgram goto_command_program = Visit(goto_command);
    ConcatenateDroneProgramCommands(drone_program, goto_command_program);
  }

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(UgvDropCommand *n) {
  DroneProgram drone_program;

  {
    GotoCommand *goto_command = new GotoCommand();
    goto_command->mutable_goal()->set_latitude(n->goal().latitude());
    goto_command->mutable_goal()->set_longitude(n->goal().longitude());
    goto_command->mutable_goal()->set_altitude(n->goal().altitude() /
                                               kFeetPerMeter);
    goto_command->set_come_to_stop(true);

    DroneProgram goto_command_program = Visit(goto_command);
    ConcatenateDroneProgramCommands(drone_program, goto_command_program);
  }

  {
    DroneCommand *bomb_drop_command = new DroneCommand();
    bomb_drop_command->mutable_trigger_bomb_drop_command();

    drone_program.mutable_commands()->AddAllocated(bomb_drop_command);
  }

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(SurveyCommand *n) {
  DroneProgram drone_program;

  // TODO(Weber): Implement
  (void)n;

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(OffAxisCommand *n) {
  DroneProgram drone_program;

  // TODO(Comran): Implement
  (void)n;

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(WaitCommand *n) {
  DroneProgram drone_program;

  {
    DroneCommand *sleep_cmd = new DroneCommand();
    SleepCommand *sleep = sleep_cmd->mutable_sleep_command();
    sleep->set_time(n->time());
    drone_program.mutable_commands()->AddAllocated(sleep_cmd);
  }

  return drone_program;
}

DroneProgram Ground2DroneVisitor::Visit(GotoCommand *n) {
  DroneProgram drone_program;

  // Go to a certain location on the field while avoiding obstacles by
  // calculating a safe path to travel and commanding the drone to follow this
  // path using TranslateCommand.

  ::lib::Position3D end = {n->goal().latitude(), n->goal().longitude(),
                           n->goal().altitude()};

  ::std::vector<::lib::Position3D> avoidance_path;

  if (!current_position_) {
    avoidance_path.push_back(end);
  } else {
    avoidance_path =
        rrt_avoidance_.Process(*current_position_, end, *ground_program_);
    delete current_position_;
  }

  // Add the path for avoiding obstacles as a list of raw goto commands,
  // which will not undergo additional lower-level rrt calculations by the
  // preprocessor.
  for (::lib::Position3D goto_step : avoidance_path) {
    DroneCommand *translate_cmd = new DroneCommand();
    TranslateCommand *translate = translate_cmd->mutable_translate_command();

    ::lib::mission_manager::Position3D *goto_raw_goal =
        new ::lib::mission_manager::Position3D();
    ::std::cout << "GOTO: " << goto_step.latitude << " " << goto_step.longitude << ::std::endl;
    goto_raw_goal->set_latitude(goto_step.latitude);
    goto_raw_goal->set_longitude(goto_step.longitude);
    goto_raw_goal->set_altitude(n->goal().altitude());

    translate->set_allocated_goal(goto_raw_goal);
    translate->set_come_to_stop(false);

    drone_program.mutable_commands()->AddAllocated(translate_cmd);
  }

  // Set current position to last command's end
  current_position_ = new ::lib::Position3D(end);

  // Return drone program
  return drone_program;
}

} // namespace ground2drone_visitor
} // namespace timeline
} // namespace ground_controls
} // namespace controls
} // namespace src
