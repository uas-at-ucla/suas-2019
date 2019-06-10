#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "zmq.hpp"
#include <boost/algorithm/string.hpp>
#include <google/protobuf/text_format.h>
#include <ros/console.h>

#include "lib/alarm/alarm.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace mission_state_machine {
namespace {
static constexpr double kAcceptanceRadius = 4.0; // meters
} // namespace

enum MissionState {
  GET_NEXT_CMD = 0,
  TRANSLATE = 1,
  UGV_DROP = 2,
  LAND = 3,
  SLEEP = 4,
};

::std::string StateToString(MissionState state);

// States //////////////////////////////////////////////////////////////////////
class State {
 public:
  State() = default;
  virtual ~State() = default;

  virtual void Handle(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output) = 0;

  virtual void Reset() = 0;
};

class TranslateState : public State {
 public:
  TranslateState();
  ~TranslateState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
  void SetSetpoints(double latitude, double longitude, double altitude);

 private:
  double setpoint_latitude_;
  double setpoint_longitude_;
  double setpoint_altitude_;
};

class UGVDropState : public State {
 public:
  UGVDropState();
  ~UGVDropState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class LandState : public State {
 public:
  LandState();
  ~LandState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class SleepState : public State {
 public:
  SleepState();
  ~SleepState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;

  void SetSleepPeriod(double sleep_period);

 private:
  double start_;
  bool was_reset_;
  double sleep_period_;
};

class UnknownState : public State {
 public:
  UnknownState();
  ~UnknownState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

// State machine router ////////////////////////////////////////////////////////
class MissionStateMachine {
 public:
  MissionStateMachine();
  ~MissionStateMachine();

  void StateTransition(::src::controls::Output &output);
  bool SafetyStateOverride(::src::controls::Goal &goal,
                           ::src::controls::Output &output);

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);

  void LoadMission(
      ::src::controls::ground_controls::timeline::DroneProgram drone_program);

 private:
  State *GetStateHandler(MissionState state);

  MissionState state_;
  ::std::map<MissionState, State *> state_handlers_;
  UnknownState *unknown_state_;

  ::src::controls::ground_controls::timeline::DroneProgram drone_program_;
  ::std::mutex drone_program_mutex_;
  int drone_program_index_;

  bool setpoint_initialized_;
  double setpoint_latitude_;
  double setpoint_longitude_;
  double setpoint_altitude_;
  double setpoint_yaw_;

  bool new_mission_ready_;
};

} // namespace mission_state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
