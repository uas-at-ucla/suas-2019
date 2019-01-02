#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "zmq.hpp"

#include <google/protobuf/util/message_differencer.h>

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/mission_message_queue/mission_message_queue.h"
#include "lib/motion_profile/motion_profile.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/pid/pid.h"
#include "lib/semaphore/semaphore.h"
#include "lib/mission_manager/mission_commands.pb.h"

// using protobuf messages defined in this namespace
using namespace lib::mission_manager;

namespace src {
namespace controls {
namespace loops {
namespace executor {

struct ExecutorOutput {
  Vector3D flight_velocities;
  float yaw;
  bool bomb_drop;
  bool alarm;
};

class Executor {
 public:
  Executor();
  ~Executor();

  ExecutorOutput Calculate(lib::Position3D position, ::Eigen::Vector3d velocity);
  void PreprocessorThread();
  void SetMission(::lib::mission_manager::Mission mission);
  ExecutorOutput VelocityNavigator();
  bool MetGoal();

  void Quit() { run_ = false; }

 private:
  ::lib::pid::PID thrust_pid_;
  ::lib::motion_profile::MotionProfile profile_;

  ::lib::mission_message_queue::MissionMessageQueueReceiver
      mission_message_queue_receiver_;

  ::lib::mission_manager::Command cmd_, last_cmd_;
  bool cmd_set_;

  lib::Position3D position_;
  bool position_set_;
  ::lib::Semaphore position_semaphore_;

  ::std::atomic<bool> run_{true};

  ::std::thread thread_;

  double sleep_time_;
  bool come_to_stop_;
  int come_to_stop_count_;

  bool setpoint_reset_;
  bool met_goal_;
  lib::Position3D start_, end_;
  ::Eigen::Vector3d current_physical_velocity_;
};

} // namespace executor
} // namespace loops
} // namespace controls
} // namespace src
