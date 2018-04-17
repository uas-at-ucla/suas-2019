#include "mission_manager.h"

namespace lib {

MissionManager::MissionManager() : semaphore_(1), command_pointer_(0) {}

void MissionManager::SetCommands(::lib::mission_manager::Mission mission) {
  semaphore_.Wait();

  mission_ = mission;
  command_pointer_ = 0;

  semaphore_.Notify();
}

void MissionManager::SetObstacles(::lib::mission_manager::Obstacles obstacles) {
  semaphore_.Wait();

  obstacles_ = obstacles;

  semaphore_.Notify();
}

void MissionManager::ClearCommands() {
  semaphore_.Wait();

  mission_ = ::lib::mission_manager::Mission();
  command_pointer_ = 0;

  semaphore_.Notify();
}

void MissionManager::PopCommand() {
  semaphore_.Wait();

  if (command_pointer_ < mission_.commands_size()) {
    command_pointer_++;
  }

  semaphore_.Notify();
}

size_t MissionManager::NumberOfCommands() { return mission_.commands_size(); }

int MissionManager::GetCurrentCommandIndex() { return command_pointer_; }

::lib::mission_manager::Command MissionManager::GetCurrentCommand() {
  // Never return a nullptr.
  if (command_pointer_ >= mission_.commands_size()) {
    ::lib::mission_manager::Command cmd;
    cmd.mutable_nothingcommand();

    return cmd;
  }

  return mission_.commands(command_pointer_);
}

void MissionManager::UnrollMission(::lib::mission_manager::Mission *mission,
                                   Position3D drone_position) {
  for (int i = 0; i < mission->commands_size(); i++) {
    ::lib::mission_manager::Command *cmd = mission->mutable_commands(i);

    ::lib::mission_manager::Mission *sub_mission = cmd->mutable_sub_mission();

    if (cmd->has_waypointcommand()) {
      ::lib::mission_manager::WaypointCommand *waypoint_cmd =
          cmd->mutable_waypointcommand();

      // Create a goto command to fly to the waypoint position.
      {
        ::lib::mission_manager::GotoCommand *goto_cmd =
            sub_mission->add_commands()->mutable_gotocommand();

        ::lib::mission_manager::Position3D *goal =
            new ::lib::mission_manager::Position3D(waypoint_cmd->goal());
        ::std::cout << goal->latitude() << ", " << goal->longitude()
                    << ::std::endl;
        goto_cmd->set_allocated_goal(goal);
      }

      // Sleep at the waypoint position once the drone arrives there.
      ::lib::mission_manager::Command *sleep_cmd_raw =
          sub_mission->add_commands();
      ::lib::mission_manager::SleepCommand *sleep_cmd =
          sleep_cmd_raw->mutable_sleepcommand();
      sleep_cmd->set_time(3);
    } else if (cmd->has_bombdropcommand()) {
      ::lib::mission_manager::BombDropCommand *bomb_cmd =
          cmd->mutable_bombdropcommand();

      // Create a waypoint command to go to the bomb drop location.
      {
        ::lib::mission_manager::WaypointCommand *waypoint_cmd =
            sub_mission->add_commands()->mutable_waypointcommand();

        ::lib::mission_manager::Position3D *goal =
            new ::lib::mission_manager::Position3D();
        goal->set_latitude(bomb_cmd->drop_zone().latitude());
        goal->set_longitude(bomb_cmd->drop_zone().latitude());
        goal->set_altitude(20);
        waypoint_cmd->set_allocated_goal(goal);
      }
    } else if (cmd->has_gotocommand()) {
      ::lib::mission_manager::GotoCommand *goto_cmd =
          cmd->mutable_gotocommand();

      Position3D end = {goto_cmd->goal().latitude(),
                        goto_cmd->goal().longitude(),
                        goto_cmd->goal().altitude()};
      ::std::vector<Position3D> avoidance_path =
          rrt_avoidance_.Process(drone_position, end, obstacles_);

      // Add the path for avoiding obstacles as a list of raw goto commands,
      // which will not undergo additional lower-level rrt calculations by the
      // preprocessor.
      for (Position3D goto_step : avoidance_path) {
        ::lib::mission_manager::GotoRawCommand *goto_raw_cmd =
            sub_mission->add_commands()->mutable_gotorawcommand();

        ::lib::mission_manager::Position3D *goto_raw_goal =
            new ::lib::mission_manager::Position3D();
        goto_raw_goal->set_latitude(goto_step.latitude);
        goto_raw_goal->set_longitude(goto_step.longitude);
        goto_raw_goal->set_altitude(goto_cmd->goal().altitude());
        goto_raw_cmd->set_allocated_goal(goto_raw_goal);
      }
    }

    // Recursive step to branch out mission even further.
    if (cmd->has_sub_mission()) {
      UnrollMission(cmd->mutable_sub_mission(), drone_position);
    }
  }
}

void MissionManager::Preprocess(Position3D drone_position) {
  // Make a copy of the mission to work with.
  semaphore_.Wait();
  ::lib::mission_manager::Mission mission;
  mission.CopyFrom(mission_);
  semaphore_.Notify();

  // Loop through all mission commands and recursively branch out non-primitive
  // commands.
  UnrollMission(&mission, drone_position);

  semaphore_.Wait();
  mission_.CopyFrom(mission);
  semaphore_.Notify();
}

void MissionManager::DumpMission() { DumpMission(mission_, 0); }

void MissionManager::DumpMission(::lib::mission_manager::Mission mission,
                                 int nest) {
  for (int j = 0; j < nest; j++) {
    ::std::cout << "    ";
  }
  ::std::cout << "MISSION\n";
  for (int i = 0; i < mission.commands_size(); i++) {
    for (int j = 0; j < nest; j++) {
      ::std::cout << "    ";
    }

    ::std::cout << "+ COMMAND ";
    if (mission.commands(i).has_waypointcommand()) {
      ::std::cout << "WAYPOINT at ("
                  << mission.commands(i).waypointcommand().goal().latitude()
                  << ", "
                  << mission.commands(i).waypointcommand().goal().longitude()
                  << ") @ alt "
                  << mission.commands(i).waypointcommand().goal().altitude()
                  << "\n";
    } else if (mission.commands(i).has_nothingcommand()) {
      ::std::cout << "NOTHING\n";
    } else if (mission.commands(i).has_gotocommand()) {
      ::std::cout << "GOTO ("
                  << mission.commands(i).gotocommand().goal().latitude() << ", "
                  << mission.commands(i).gotocommand().goal().longitude()
                  << ") @ alt "
                  << mission.commands(i).gotocommand().goal().altitude()
                  << "\n";
    } else if (mission.commands(i).has_gotorawcommand()) {
      ::std::cout << "GOTO RAW at ("
                  << mission.commands(i).gotorawcommand().goal().latitude()
                  << ", "
                  << mission.commands(i).gotorawcommand().goal().longitude()
                  << ") @ alt "
                  << mission.commands(i).gotorawcommand().goal().altitude()
                  << "\n";
    } else if (mission.commands(i).has_sleepcommand()) {
      ::std::cout << "SLEEP for " << mission.commands(i).sleepcommand().time()
                  << " seconds\n";
    } else if (mission.commands(i).has_bombdropcommand()) {
      ::std::cout
          << "BOMB DROP at ("
          << mission.commands(i).bombdropcommand().drop_zone().latitude() << ", "
          << mission.commands(i).bombdropcommand().drop_zone().longitude()
          << ")\n";
    } else {
      ::std::cout << "\n";
    };

    if (mission.commands(i).has_sub_mission()) {
      DumpMission(mission.commands(i).sub_mission(), nest + 1);
    }
  }
}

}  // namespace lib
