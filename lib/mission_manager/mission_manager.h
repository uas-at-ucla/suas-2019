#ifndef SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
#define SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_

#include <mutex>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <vector>

#include "lib/mission_manager/mission_commands.pb.h"

namespace lib {

class Semaphore {
 public:
  Semaphore(unsigned long count);
  void Notify();
  void Wait();
  bool TryWait();

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned long count_;
};

class MissionManager {
 public:
  MissionManager();
  void SetCommands(::lib::mission_manager::Mission mission);
  void ClearCommands();
  void PopCommand();
  size_t NumberOfCommands();

  int GetCurrentCommandIndex();
  ::lib::mission_manager::Command GetCurrentCommand();

 private:
  ::lib::mission_manager::Mission mission_;

  Semaphore semaphore_;
  int command_pointer_;
};

}  // namespace lib

#endif  // SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
