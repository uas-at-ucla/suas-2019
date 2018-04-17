#include "mission_manager.h"

namespace lib {

MissionManager::MissionManager() : semaphore_(1), command_pointer_(0) {}

void MissionManager::SetCommands(::lib::mission_manager::Mission mission) {
  semaphore_.Wait();

  mission_ = mission;
  command_pointer_ = 0;

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

void MissionManager::UnrollMission(::lib::mission_manager::Mission *mission) {
  for (int i = 0; i < mission->commands_size(); i++) {
    ::lib::mission_manager::Command *cmd = mission->mutable_commands(i);

    if (cmd->has_waypointcommand()) {
      ::lib::mission_manager::WaypointCommand *waypoint_cmd =
          cmd->mutable_waypointcommand();

      ::lib::mission_manager::Mission *sub_mission = cmd->mutable_sub_mission();

      {
        // Create a goto command to fly to the waypoint position.
        ::lib::mission_manager::Command *goto_cmd_raw =
            sub_mission->add_commands();
        ::lib::mission_manager::GotoCommand *goto_cmd =
            goto_cmd_raw->mutable_gotocommand();

        ::lib::mission_manager::Position3D *goal =
            new ::lib::mission_manager::Position3D(waypoint_cmd->goal());
        goto_cmd->set_allocated_goal(goal);
      }

      // Sleep at the waypoint position once the drone arrives there.
      ::lib::mission_manager::Command *sleep_cmd_raw =
          sub_mission->add_commands();
      ::lib::mission_manager::SleepCommand *sleep_cmd =
          sleep_cmd_raw->mutable_sleepcommand();
      sleep_cmd->set_time(3);
    }

    // Recursive step to branch out mission even further.
    if (cmd->has_sub_mission()) {
      UnrollMission(cmd->mutable_sub_mission());
    }
  }
}

void MissionManager::Preprocess() {
  // Make a copy of the mission to work with.
  semaphore_.Wait();
  ::lib::mission_manager::Mission mission;
  mission.CopyFrom(mission_);
  semaphore_.Notify();

  // Loop through all mission commands and recursively branch out non-primitive
  // commands.
  UnrollMission(&mission);

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
    if(mission.commands(i).has_waypointcommand()) {
      ::std::cout << "WAYPOINT\n";
    } else if(mission.commands(i).has_nothingcommand()) {
      ::std::cout << "NOTHING\n";
    } else if(mission.commands(i).has_gotocommand()) {
      ::std::cout << "GOTO\n";
    } else if(mission.commands(i).has_sleepcommand()) {
      ::std::cout << "SLEEP\n";
    } else {
      ::std::cout << "\n";
    };

    if (mission.commands(i).has_sub_mission()) {
      DumpMission(mission.commands(i).sub_mission(), nest + 1);
    }
  }
}

}  // namespace lib
