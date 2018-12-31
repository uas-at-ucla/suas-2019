#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "zmq.hpp"
#include <boost/algorithm/string.hpp>
#include <google/protobuf/text_format.h>

#include "lib/alarm/alarm.h"
#include "lib/logger/log_sender.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/controls/ground_server/timeline/executor/executor.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

enum FlightLoopState {
  STANDBY,
  ARMING,
  ARMED_WAIT_FOR_SPINUP,
  ARMED,
  TAKING_OFF,
  TAKEN_OFF,
  SAFETY_PILOT_CONTROL,
  MISSION,
  LANDING,
  FAILSAFE,
  FLIGHT_TERMINATION,
};

::std::string StateToString(FlightLoopState state);

// States //////////////////////////////////////////////////////////////////////
class State {
 public:
  State() {}
  virtual void Handle(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output) = 0;

  virtual void Reset() = 0;
};

class StandbyState : public State {
 public:
  StandbyState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class ArmingState : public State {
 public:
  ArmingState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class ArmedWaitForSpinupState : public State {
 public:
  ArmedWaitForSpinupState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class ArmedState : public State {
 public:
  ArmedState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class TakingOffState : public State {
 public:
  TakingOffState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class TakenOffState : public State {
 public:
  TakenOffState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class SafetyPilotControlState : public State {
 public:
  SafetyPilotControlState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class MissionState : public State {
 public:
  MissionState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();

 private:
  executor::Executor executor_;
};

class LandingState : public State {
 public:
  LandingState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class FailsafeState : public State {
 public:
  FailsafeState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

class FlightTerminationState : public State {
 public:
  FlightTerminationState();
  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);
  void Reset();
};

// State machine router ////////////////////////////////////////////////////////
class StateMachine {
 public:
  StateMachine();

  void StateTransition(::src::controls::Output &output);
  bool SafetyStateOverride(::src::controls::Goal &goal,
                           ::src::controls::Output &output);

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);

 private:
  FlightLoopState state_;

  StandbyState standby_state_;
  ArmingState arming_state_;
  ArmedWaitForSpinupState armed_wait_for_spinup_state_;
  ArmedState armed_state_;
  TakingOffState taking_off_state_;
  TakenOffState taken_off_state_;
  SafetyPilotControlState safety_pilot_control_state_;
  MissionState mission_state_;
  LandingState landing_state_;
  FailsafeState failsafe_state_;
  FlightTerminationState flight_termination_state_;
};

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
