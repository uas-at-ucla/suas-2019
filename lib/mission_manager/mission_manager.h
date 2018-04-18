#ifndef SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
#define SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_

#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/rrt_avoidance/rrt_avoidance.h"

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
  void SetObstacles(::lib::mission_manager::Obstacles obstacles);

  void ClearCommands();
  void PopCommand();
  size_t NumberOfCommands();

  int GetCurrentCommandIndex();
  ::lib::mission_manager::Command GetCurrentCommand();
  void Preprocess(Position3D drone_position);
  void DumpMission();

  void UnrollMission(
      ::lib::mission_manager::Mission *mission, Position3D drone_position);

 private:
  rrt_avoidance::RRTAvoidance rrt_avoidance_;

  ::lib::mission_manager::Mission mission_;
  ::lib::mission_manager::Obstacles obstacles_;

  void DumpMission(::lib::mission_manager::Mission, int nest);

  Semaphore semaphore_;
  int command_pointer_;
};

}  // namespace lib

#endif  // SPINNY_LIB_MISSION_MANAGER_MISSION_MANAGER_H_
