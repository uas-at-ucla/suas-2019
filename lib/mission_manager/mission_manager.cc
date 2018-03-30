#include "mission_manager.h"

namespace lib {

Semaphore::Semaphore(unsigned long count) : count_(count) {}

void Semaphore::Notify() {
  std::unique_lock<decltype(mutex_)> lock(mutex_);
  count_++;
  condition_.notify_one();
}

void Semaphore::Wait() {
  std::unique_lock<decltype(mutex_)> lock(mutex_);
  while (!count_)  // Handle spurious wake-ups.
    condition_.wait(lock);
  count_--;
}

bool Semaphore::TryWait() {
  std::unique_lock<decltype(mutex_)> lock(mutex_);
  if (count_) {
    count_--;
    return true;
  }

  return false;
}

MissionCommand::MissionCommand() {}

MissionCommandGoto::MissionCommandGoto(double latitude, double longitude,
                                       double altitude) {
  latitude_ = latitude;
  longitude_ = longitude;
  altitude_ = altitude;
}

MissionCommandGoto::MissionCommandGoto(MissionCommandGoto* cmd) {
  latitude_ = cmd->latitude();
  longitude_ = cmd->longitude();
  altitude_ = cmd->altitude();
}

MissionCommandBombDrop::MissionCommandBombDrop() {}
MissionCommandBombDrop::MissionCommandBombDrop(MissionCommandBombDrop* cmd) {
  (void)cmd;
}

MissionCommandDoNothing::MissionCommandDoNothing() {}
MissionCommandDoNothing::MissionCommandDoNothing(MissionCommandDoNothing* cmd) {
  (void)cmd;
}

MissionManager::MissionManager() : semaphore_(1), command_pointer_(0) {}

void MissionManager::AddCommands(
    ::std::vector<::std::shared_ptr<MissionCommand>> new_commands) {
  semaphore_.Wait();

  commands_.insert(commands_.end(), new_commands.begin(), new_commands.end());

  semaphore_.Notify();
}

void MissionManager::ClearCommands() {
  semaphore_.Wait();

  commands_.clear();
  command_pointer_ = 0;

  semaphore_.Notify();
}

void MissionManager::PopCommand() {
  semaphore_.Wait();

  if (command_pointer_ < commands_.size()) {
    command_pointer_++;
  }

  semaphore_.Notify();
}

size_t MissionManager::NumberOfCommands() { return commands_.size(); }

::std::shared_ptr<MissionCommand> MissionManager::GetCurrentCommand() {
  // Never return a nullptr.
  if(!commands_.size()) {
    return ::std::shared_ptr<MissionCommand>(new MissionCommandDoNothing());
  }

  return commands_[command_pointer_];
}

}  // namespace lib
