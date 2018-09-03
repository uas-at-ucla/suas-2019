#pragma once

#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/rrt_avoidance/rrt_avoidance.h"
#include "lib/semaphore/semaphore.h"

namespace lib {

class MissionManager {
 public:
  MissionManager();
  void SetCommands(::lib::mission_manager::Mission mission);
  void SetObstacles(::lib::mission_manager::Obstacles obstacles);

  void ClearCommands();
  bool CheckNewCommand();
  void PopCommand();
  size_t NumberOfCommands();

  ::lib::mission_manager::Command GetCurrentCommand();
  ::lib::mission_manager::Mission GetMission();
  void Preprocess(Position3D drone_position);
  void DumpMission();

  void UnrollMission(::lib::mission_manager::Mission *mission,
                     Position3D drone_position);

 private:
  rrt_avoidance::RRTAvoidance rrt_avoidance_;

  ::lib::mission_manager::Mission mission_;
  ::lib::mission_manager::Obstacles obstacles_;

  ::lib::mission_manager::Command
  GetCurrentCommand(::lib::mission_manager::Mission &mission);
  bool PopCommand(::lib::mission_manager::Mission &mission);
  void DumpMission(::lib::mission_manager::Mission, int nest);

  bool new_command_;

  Semaphore semaphore_;
};

} // namespace lib
