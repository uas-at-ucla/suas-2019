#include "mission_manager.h"

namespace lib {

MissionManager::MissionManager() : semaphore_(1), command_pointer_(0) {}

void MissionManager::SetCommands(
    ::lib::mission_manager::Mission mission) {
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
    cmd.mutable_nothing_command();

    return cmd;
  }

  return mission_.commands(command_pointer_);
}

}  // namespace lib
